"""
PyDobotController - pydobotライブラリを使用したDobot制御クラス

pydobotはシリアル通信を直接実装しているため、各インスタンスが独立した接続を持ち、
複数のDobotを同時に制御できます。

使用方法:
    from src.devices.dobot.pydobot_controller import PyDobotController

    # 2台のDobotを同時接続
    dobot1 = PyDobotController(port_name='COM8')
    dobot2 = PyDobotController(port_name='COM9')

    # 各Dobotを独立して制御
    dobot1.move_XYZ_abs(200, 0, 100)
    dobot2.move_XYZ_abs(200, 50, 100)

    # 終了時
    dobot1.disconnect()
    dobot2.disconnect()

"""

from typing import Optional, List
import logging
import struct
import time

logger = logging.getLogger(__name__)

try:
    from pydobot import Dobot
    from pydobot.enums import PTPMode
    PYDOBOT_AVAILABLE = True
except ImportError:
    PYDOBOT_AVAILABLE = False
    PTPMode = None
    logger.warning("Warning: pydobot not installed. Run 'pip install pydobot'")

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

from .dobot_config import DobotConfig
from . import pydobot_patch
from .pydobot_patch import apply_pydobot_patch
apply_pydobot_patch()


class PyDobotController:
    """
    pydobotベースのDobot制御クラス

    既存のDobotControllerと同じAPIを提供しつつ、
    pydobotライブラリを使用することで複数インスタンスの同時利用が可能。
    """

    def __init__(self, port_name: str = None, homing: bool = False, speed_preset: str = None,
                 verbose: bool = False, move_timeout_s: Optional[float] = None):
        """
        PyDobotControllerを初期化する

        Args:
            port_name (str): Dobotが接続されているシリアルポート名
            homing (bool): 初期化時にホーミングを実行するかどうか
            speed_preset (str): 使用する速度プリセット名
            verbose (bool): デバッグ出力を有効にするかどうか
            move_timeout_s (float): キュー付き移動 1 回の完了待ちの上限秒。
                省略時は pydobot_patch.DEFAULT_MOVE_TIMEOUT_S (60 s)。超えると
                pydobot_patch.DobotWaitTimeout（アームは止めないので呼び出し側で
                force_stop すること）。
        """
        if not PYDOBOT_AVAILABLE:
            raise ImportError("pydobot library is not installed. Run 'pip install pydobot'")

        if port_name is None:
            port_name = DobotConfig.CONNECTION_SETTINGS["default_port"]

        self.port_name = port_name
        self.speed_preset = speed_preset or DobotConfig.DEFAULT_SPEED
        self.current_speed_config = DobotConfig.get_speed_preset(self.speed_preset)
        self.verbose = verbose
        self.move_timeout_s = move_timeout_s

        self.device: Optional[Dobot] = None
        self._connected = False

        logger.info(f"Using speed preset: {self.speed_preset}")
        logger.info(f"Speed description: {self.current_speed_config['description']}")

        self._initialize(homing)

    def _initialize(self, homing: bool):
        """Dobotに接続して初期化する"""
        try:
            logger.info(f"Connecting to Dobot on {self.port_name}...")
            logger.info(f"Verbose mode: {self.verbose}")
            self.device = Dobot(port=self.port_name, verbose=self.verbose)
            if self.move_timeout_s is not None:
                self.device.move_timeout_s = float(self.move_timeout_s)
            self._connected = True
            logger.info(f"接続状態: DobotConnect_NoError")

            # 速度設定（直交・Joint）を適用
            self._apply_speed(self.current_speed_config)

            if homing:
                # pydobot 1.3.x には home() が無いので、自前の home() を使う
                logger.info("Executing homing...")
                self.home()

        except Exception as e:
            logger.error(f"Failed to connect to Dobot on {self.port_name}: {e}")
            self._connected = False
            if self.device is not None:
                # 閉じずに捨てると Windows では COM ポートが掴まれたままになる
                try:
                    self.device.close()
                except Exception as close_err:
                    logger.error(f"Failed to close {self.port_name} after init error: {close_err}")
            self.device = None
            # 接続失敗を握りつぶさず送出する。以前は例外を飲み込んで device=None のまま
            # 継続していたため、以降の move_* が無音で何もせず「成功」を装う危険があった。
            raise ConnectionError(
                f"Dobot への接続に失敗しました (port={self.port_name}): {e}"
            ) from e

    @property
    def api(self):
        """
        互換性のためのプロパティ。
        接続状態を確認するために使用（Noneでなければ接続済み）
        """
        return self.device if self._connected else None

    def __enter__(self):
        """コンテキストマネージャーとして使用時の処理"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャーとして使用時の切断処理"""
        self.disconnect()

    def _require_device(self) -> "Dobot":
        """未接続の場合は例外を送出する。

        以前は各メソッドが ``if self.device:`` で未接続を黙ってスキップしていたため、
        接続断（や初期化失敗）に気付かず空動作したまま処理が進む危険があった。
        安全のため、未接続での操作要求は明示的にエラーとする。
        """
        if self.device is None or not self._connected:
            raise ConnectionError(
                f"Dobot が接続されていません (port={self.port_name})。操作を中止します。"
            )
        return self.device

    def _apply_speed(self, speed_config: dict) -> None:
        """直交 PTP パラメータと Joint PTP パラメータの両方を設定する。

        Joint 側は MOVJ_ANGLE（move_angle / rotate_relative）の回転速度に効く。
        """
        self.device.speed(
            velocity=speed_config["linear_velocity"],
            acceleration=speed_config["linear_acceleration"]
        )
        jv = speed_config["joint_velocity"]
        ja = speed_config["joint_acceleration"]
        self.device._set_ptp_joint_params(*jv[:4], *ja[:4])

    def _checked_send(self, msg, what: str, wait: bool = False):
        """_send_command の応答 None を「未確認」として ConnectionError にする。

        パッチ済み pydobot は応答が無ければ自ら例外を出すが、None を返す実装
        （未パッチ・フェイク）でも成功扱いにしない。
        """
        response = self.device._send_command(msg, wait=wait) if wait else self.device._send_command(msg)
        if response is None:
            logger.warning(f"[PyDobot] {what}: no reply from {self.port_name}; result unconfirmed")
            raise ConnectionError(f"Dobot: {what} に応答がありません（実行を確認できません）")
        return response

    @property
    def stop_requested(self) -> bool:
        """force_stop() 後、次の移動指示まで True。"""
        return self.device is not None and pydobot_patch.stop_requested(self.device)

    def _safe_pose(self, max_retries=3, retry_delay=0.2):
        """pose() のリトライラッパー。一時的なシリアル通信障害に対応。"""
        for attempt in range(max_retries):
            try:
                result = self.device.pose()
                if result is not None:
                    return result
            except (AttributeError, struct.error, pydobot_patch.DobotReplyError) as e:
                if attempt == max_retries - 1:
                    raise ConnectionError(
                        f"Dobot serial communication failed after {max_retries} attempts"
                    ) from e
            time.sleep(retry_delay * (attempt + 1))
        raise ConnectionError("Dobot serial communication timeout")

    def move_XYZ_abs(self, x_pos: float, y_pos: float, z_pos: float, r: float = None):
        """
        絶対座標でロボットを移動させる

        Args:
            x_pos (float): X座標
            y_pos (float): Y座標
            z_pos (float): Z座標
            r (float): 回転角度（省略時は現在の角度を維持）
        """
        self._require_device()
        if r is None:
            pose = self._safe_pose()
            r = pose[3]
        if self.verbose:
            logger.info(f"[PyDobot] move_XYZ_abs: X={x_pos:.1f}, Y={y_pos:.1f}, Z={z_pos:.1f}, R={r:.1f}")
        self.device.move_to(x_pos, y_pos, z_pos, r, wait=True)
        if self.verbose:
            new_pose = self._safe_pose()
            logger.info(f"[PyDobot] After move: X={new_pose[0]:.1f}, Y={new_pose[1]:.1f}, Z={new_pose[2]:.1f}")

    def move_Z(self, z_offset: float):
        """
        Z方向のみ相対移動する

        Args:
            z_offset (float): Z方向の移動量
        """
        self._require_device()
        pose = self._safe_pose()
        self.device.move_to(pose[0], pose[1], pose[2] + z_offset, pose[3], wait=True)

    def move_angle(self, angle: float):
        """
        ロボットのベース回転角度を設定する（ジョイント角度モード使用）

        Args:
            angle (float): 目標回転角度（度）- J1（ベース）の絶対角度。
                符号の規約は 正=反時計回り（上から見て、+Y方向）、負=時計回り
                （Dobot Magician の J1 正方向）。
        """
        self._require_device()
        pose = self._safe_pose()
        # pose = [x, y, z, r, j1, j2, j3, j4]
        current_j1 = pose[4]
        j2 = pose[5]
        j3 = pose[6]
        j4 = pose[7]

        if self.verbose:
            logger.info(f"[PyDobot] move_angle (MOVJ_ANGLE): current J1={current_j1:.1f} -> target J1={angle:.1f}")

        # J4 も J1 と逆に回して r（= J1 + J4）を保つ。XYZ 移動は r を保つので、
        # ここで J4 を保つと「J1 回転 → XYZ 移動」のたびに J4 が ΔJ1 ずつたまり、
        # やがて J4 の可動域（約 ±150°）を超えて alarm 0x22 になる。
        target_j4 = j4 - (angle - current_j1)

        # MOVJ_ANGLEモードでジョイント角度を直接指定
        # _set_ptp_cmd(j1, j2, j3, j4, mode, wait)
        self.device._set_ptp_cmd(angle, j2, j3, target_j4, mode=PTPMode.MOVJ_ANGLE, wait=True)

        if self.verbose:
            new_pose = self._safe_pose()
            logger.info(f"[PyDobot] After move_angle: J1={new_pose[4]:.1f}")

    def move_XY(self, x_offset: float, y_offset: float):
        """
        XY平面上で相対移動する

        Args:
            x_offset (float): X方向の移動量
            y_offset (float): Y方向の移動量
        """
        self._require_device()
        if x_offset == 0 and y_offset == 0:
            return
        pose = self._safe_pose()
        self.device.move_to(
            pose[0] + x_offset,
            pose[1] + y_offset,
            pose[2],
            pose[3],
            wait=True
        )

    def move_to_initial_pos(self):
        """初期位置に移動する"""
        self.move_XYZ_abs(200, 0, 100)
        self.move_angle(0)

    def pickup(self, x_offset: float, y_offset: float, z_offset: float):
        """
        ピックアップ動作を行う

        Args:
            x_offset (float): X方向の移動量
            y_offset (float): Y方向の移動量
            z_offset (float): Z方向の移動量
        """
        self.move_XY(x_offset, y_offset)
        self.move_Z(z_offset)

    def place(self, x_offset: float, y_offset: float, z_offset: float):
        """
        配置動作を行う

        Args:
            x_offset (float): X方向の移動量
            y_offset (float): Y方向の移動量
            z_offset (float): Z方向の移動量
        """
        self.move_Z(-z_offset)
        self.move_XY(-x_offset, y_offset)

    def force_stop(self) -> bool:
        """コマンドキューを強制停止し、残キューを破棄する（緊急停止）。

        Dobot プロトコルの QueuedCmdForceStopExec (ID:242) を直接送信する。
        StopExec (ID:241) と異なり、実行中の移動コマンドも即時に停止する。
        併せて残キューを破棄し、コンベア（EMotor）も即時停止する。

        緊急停止経路であるため、未接続や通信エラーでも例外を送出しない
        （ベストエフォート）。

        Returns:
            bool: すべての停止コマンドの送信に成功した場合 True
        """
        if self.device is None:
            logger.error(f"[PyDobot] force_stop: not connected ({self.port_name})")
            return False

        from pydobot.message import Message
        ok = True

        # 0. 完了待ち中のスレッド（move_* の wait=True や home()）を即座に解放する。
        #    停止コマンドより先に立てる（キュー破棄で index が期待値に届かなくなるため）。
        try:
            pydobot_patch.request_stop(self.device)
        except Exception as e:
            logger.error(f"[PyDobot] force_stop: could not set stop flag: {e}")

        # 1. QueuedCmdForceStopExec (ID:242, isQueued=0): 実行中コマンドも即時停止
        try:
            msg = Message()
            msg.id = 242
            msg.ctrl = 0x01
            msg.params = bytearray([])
            if self.device._send_command(msg) is None:
                raise ConnectionError("no reply to QueuedCmdForceStopExec")
            logger.info(f"[PyDobot] force_stop: queue force-stopped ({self.port_name})")
        except Exception as e:
            ok = False
            logger.error(f"[PyDobot] force_stop: ForceStopExec failed: {e}")

        # 2. 残キューを破棄（再開時に停止前の動作が走らないようにする）
        try:
            if self.device._set_queued_cmd_clear() is None:
                raise ConnectionError("no reply to QueuedCmdClear")
        except Exception as e:
            ok = False
            logger.error(f"[PyDobot] force_stop: queue clear failed: {e}")

        # 3. コンベア（EMotor, ID:135, isQueued=0）を即時停止
        for index in (0, 1):
            try:
                msg = Message()
                msg.id = 135
                msg.ctrl = 0x01
                msg.params = bytearray([index, 0x00])  # index, isEnabled=0
                msg.params.extend(bytearray(struct.pack('i', 0)))  # speed=0
                if self.device._send_command(msg) is None:
                    raise ConnectionError("no reply")
            except Exception as e:
                ok = False
                logger.error(f"[PyDobot] force_stop: EMotor {index} stop failed: {e}")

        return ok

    def disconnect(self):
        """Dobotとの接続を切断する"""
        if self.device:
            try:
                self.device.close()
                logger.info(f"Disconnected from {self.port_name}")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self._connected = False
                self.device = None

    def get_current_position(self) -> List[float]:
        """
        現在のロボットの位置と角度を取得する

        Returns:
            list: [x, y, z, r, joint1, joint2, joint3, joint4]
        """
        self._require_device()
        return list(self._safe_pose())

    def set_gripper(self, enabled: bool, on: bool):
        """
        グリッパーの状態を設定する

        Args:
            enabled (bool): グリッパーの制御を有効にするかどうか
            on (bool): グリッパーをオン（閉じる）にするかオフ（開く）にするか

        Returns:
            int: 0（互換性のため）
        """
        self._require_device()
        # pydobot の grip() は応答を捨てるので、下位メソッドで応答を確認する
        if self.device._set_end_effector_gripper(bool(on)) is None:
            logger.warning(f"[PyDobot] set_gripper: no reply from {self.port_name}; state unconfirmed")
            raise ConnectionError("Dobot: グリッパー設定に応答がありません（状態を確認できません）")
        return 0

    def set_suction_cup(self, enabled: bool, on: bool):
        """
        吸引カップの状態を設定する

        Args:
            enabled (bool): 吸引カップの制御を有効にするかどうか
            on (bool): 吸引カップをオンにするかどうか

        Returns:
            int: 0（互換性のため）
        """
        self._require_device()
        if self.device._set_end_effector_suction_cup(bool(on)) is None:
            logger.warning(f"[PyDobot] set_suction_cup: no reply from {self.port_name}; state unconfirmed")
            raise ConnectionError("Dobot: 吸引カップ設定に応答がありません（状態を確認できません）")
        return 0

    def set_speed_preset(self, speed_preset: str):
        """
        速度プリセットを変更する

        Args:
            speed_preset (str): 新しい速度プリセット名
        """
        self.speed_preset = speed_preset
        self.current_speed_config = DobotConfig.get_speed_preset(speed_preset)

        logger.info(f"Speed preset changed to: {self.speed_preset}")
        logger.info(f"Description: {self.current_speed_config['description']}")

        if self.device is None:
            logger.warning(
                f"[PyDobot] set_speed_preset: 未接続のため速度設定は保持のみ ({self.port_name})"
            )
        else:
            # 直交だけでなく Joint も設定する（move_angle / rotate_relative の速度に効く）
            self._apply_speed(self.current_speed_config)

    # Dobot プロトコル ID（Dobot Magician Communication Protocol v1.1.5）
    _ID_SET_HOME_PARAMS = 30   # SetHOMEParams: ホーミング後に戻る座標 (x, y, z, r)
    _ID_SET_HOME_CMD = 31      # SetHOMECmd:    ホーミング（原点復帰）を実行

    def home(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        r: Optional[float] = None,
        timeout_s: float = 120.0,
        poll_interval: float = 0.5,
    ) -> List[float]:
        """ファームウェアのホーミング（原点復帰）を実行し、完了まで待つ。

        電源投入後に一度実行すると、エンコーダ基準の関節角が実機と一致し、
        XYZ 直線補間（MOVL）が正しい鉛直・水平になる。pydobot 1.3.x には
        ホーミング API が無いため、Dobot プロトコルの SetHOMEParams (ID 30) と
        SetHOMECmd (ID 31) をキュー付きで直接送信する。

        注意:
            ホーミング中はアームが J1 の限界まで大きく振れる。周囲を空けてから
            呼ぶこと。ホーミングそのものの軌道はファームウェアが決めるので
            WorkspaceValidator では検証できない。検証できるのは戻り先
            (x, y, z, r) だけであり、それは呼び出し側（home_dobot CLI）が行う。

        Args:
            x, y, z, r: ホーミング完了後にアームが戻る座標。省略した軸は
                呼び出し時点の現在値を使う（既定: 開始位置に戻る）。
            timeout_s: 完了待ちの上限秒。超えると TimeoutError。
            poll_interval: キュー進捗のポーリング間隔（秒）。

        Returns:
            list: 完了後の [x, y, z, r, joint1, joint2, joint3, joint4]

        Raises:
            ConnectionError: 未接続、またはコマンドに応答が無い
            TimeoutError: timeout_s 以内にホーミングが完了しない
        """
        device = self._require_device()
        from pydobot.message import Message

        pose = self._safe_pose()
        target = [
            pose[0] if x is None else float(x),
            pose[1] if y is None else float(y),
            pose[2] if z is None else float(z),
            pose[3] if r is None else float(r),
        ]
        logger.info(
            f"[PyDobot] homing: start pose X={pose[0]:.1f} Y={pose[1]:.1f} "
            f"Z={pose[2]:.1f} R={pose[3]:.1f}; return target X={target[0]:.1f} "
            f"Y={target[1]:.1f} Z={target[2]:.1f} R={target[3]:.1f}"
        )

        # 1. SetHOMEParams (ID 30, rw=1, isQueued=1): ホーミング後の戻り先
        msg = Message()
        msg.id = self._ID_SET_HOME_PARAMS
        msg.ctrl = 0x03
        msg.params = bytearray()
        for v in target:
            msg.params.extend(bytearray(struct.pack('<f', v)))
        if device._send_command(msg) is None:
            raise ConnectionError("Dobot: SetHOMEParams に応答がありません")

        # 2. SetHOMECmd (ID 31, rw=1, isQueued=1): 応答はキュー index (uint64)
        msg = Message()
        msg.id = self._ID_SET_HOME_CMD
        msg.ctrl = 0x03
        msg.params = bytearray(struct.pack('<I', 0))  # reserved
        pydobot_patch.clear_stop(device)   # 新しい動作の指示 → 以前の停止要求を下ろす
        response = device._send_command(msg)
        if response is None or len(response.params) < 4:
            raise ConnectionError("Dobot: SetHOMECmd に応答がありません")
        raw = bytes(response.params[:8]).ljust(8, b"\x00")
        expected_idx = struct.unpack('<Q', raw)[0]
        logger.info(f"[PyDobot] homing started (queue index {expected_idx}); the arm will swing widely")

        # 3. キューの現在 index が SetHOMECmd の index に達するまで待つ
        t0 = time.monotonic()
        while True:
            if pydobot_patch.stop_requested(device):
                raise pydobot_patch.DobotMoveAborted("Dobot: homing wait aborted by force_stop")
            current_idx = device._get_queued_cmd_current_index()
            if current_idx >= expected_idx:
                break
            if time.monotonic() - t0 > timeout_s:
                raise TimeoutError(
                    f"Dobot: ホーミングが {timeout_s:.0f}s 以内に完了しません "
                    f"(queue index {current_idx} / {expected_idx})"
                )
            time.sleep(poll_interval)
        time.sleep(min(1.0, max(poll_interval, 0.0)))

        final = list(self._safe_pose())
        logger.info(
            f"[PyDobot] homing done in {time.monotonic() - t0:.0f}s: "
            f"X={final[0]:.1f} Y={final[1]:.1f} Z={final[2]:.1f} R={final[3]:.1f} "
            f"J1={final[4]:.1f} J2={final[5]:.1f} J3={final[6]:.1f} J4={final[7]:.1f}"
        )
        return final

    def set_home_params(self, x: float, y: float, z: float, r: float) -> bool:
        """
        ホームポジションのパラメータを設定する

        Args:
            x (float): ホーム位置のX座標
            y (float): ホーム位置のY座標
            z (float): ホーム位置のZ座標
            r (float): ホーム位置の回転角度（度）

        Returns:
            bool: 設定成功時True
        """
        # pydobotにはホームパラメータ設定機能がないため、
        # ホーム位置として記録するのみ
        self._home_params = {"x": x, "y": y, "z": z, "r": r}
        logger.info(f"Home parameters set: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, R={r:.1f}")
        return True

    def list_available_speeds(self):
        """利用可能な速度設定を表示する"""
        DobotConfig.list_speed_presets()

    def list_available_positions(self):
        """利用可能な作業位置を表示する"""
        DobotConfig.list_work_positions()

    @staticmethod
    def list_available_ports() -> List[str]:
        """
        利用可能なCOMポートの一覧を取得する

        Returns:
            list: 利用可能なポート名のリスト
        """
        if list_ports:
            return [p.device for p in list_ports.comports()]
        return []

    def move_slider(self, slider_pos):
        """
        スライダー（リニアレール）を指定位置に移動する

        pydobotの内部シリアル通信を使用し、Dobotプロトコルの
        SetDeviceWithL (ID:3) + SetPTPWithLCmd (ID:86) を直接送信する。

        Args:
            slider_pos (float): スライダー目標位置 (mm)
        """
        self._require_device()
        from pydobot.message import Message

        # 1. L軸モードを有効化 (SetDeviceWithL: コマンドID 3)
        msg = Message()
        msg.id = 3
        msg.ctrl = 0x03
        msg.params = bytearray([0x01, 0x00])  # isWithL=1, version=0
        self._checked_send(msg, "SetDeviceWithL")

        # 2. 現在のアーム位置を取得
        pose = self._safe_pose()

        # 3. アーム姿勢を維持したままスライダーを移動
        #    (SetPTPWithLCmd: コマンドID 86)
        msg = Message()
        msg.id = 86
        msg.ctrl = 0x03
        msg.params = bytearray([0x01])  # ptpMode = MOVJ_XYZ
        msg.params.extend(bytearray(struct.pack('f', pose[0])))   # x
        msg.params.extend(bytearray(struct.pack('f', pose[1])))   # y
        msg.params.extend(bytearray(struct.pack('f', pose[2])))   # z
        msg.params.extend(bytearray(struct.pack('f', pose[3])))   # rHead
        msg.params.extend(bytearray(struct.pack('f', slider_pos)))  # l
        self._checked_send(msg, "SetPTPWithLCmd", wait=True)

        if self.verbose:
            logger.info(f"[PyDobot] move_slider: position={slider_pos:.1f}")

    def move_conveyer(self, index, speed, time_seconds):
        """
        コンベアベルトを動かす

        pydobotの内部シリアル通信を使用し、Dobotプロトコルの
        SetEMotor (ID:135) を直接送信する。

        Args:
            index (int): コンベアベルトのインデックス (0 or 1)
            speed (float): 速度 (mm/s)
            time_seconds (float): 動作時間（秒）
        """
        self._require_device()
        from pydobot.message import Message

        # 速度をステッピングモーターのステップ数に変換
        speed /= 2
        STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
        MM_PER_CRICLE = 3.1415926535898 * 36.0
        vel = float(speed) * STEP_PER_CRICLE / MM_PER_CRICLE

        # モーター開始 (SetEMotor: コマンドID 135)
        msg = Message()
        msg.id = 135
        msg.ctrl = 0x03
        msg.params = bytearray([index, 0x01])  # index, isEnabled=1
        msg.params.extend(bytearray(struct.pack('i', int(vel))))  # speed (int32)
        self._checked_send(msg, "SetEMotor (start)")

        # 指定時間待機
        time.sleep(time_seconds)

        # モーター停止
        msg = Message()
        msg.id = 135
        msg.ctrl = 0x03
        msg.params = bytearray([index, 0x00])  # index, isEnabled=0
        msg.params.extend(bytearray(struct.pack('i', 0)))  # speed=0
        self._checked_send(msg, f"SetEMotor (stop, conveyor {index} may still run)")

        if self.verbose:
            logger.info(f"[PyDobot] move_conveyer: index={index}, speed={speed*2:.1f}, time={time_seconds:.1f}s")

    # 注意: 以下の機能はpydobotでは対応していません
    # - get_device_sn: シリアル番号取得
    #
    # これが必要な場合は、公式DobotControllerを使用してください。


if __name__ == '__main__':
    """
    PyDobotControllerクラスの使用例
    """
    print("Available ports:", PyDobotController.list_available_ports())

    homing = False

    try:
        print("\nConnecting to Dobot...")

        # 接続テスト
        dobot = PyDobotController(port_name='COM8', homing=homing)

        if dobot.api is None:
            print("ERROR: Failed to connect to Dobot.")
            exit(1)

        print("Successfully connected!")

        # 現在位置を取得
        current_pos = dobot.get_current_position()
        print(f"Current position: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")

        # 速度設定一覧
        print("\n=== Available Speeds ===")
        dobot.list_available_speeds()

        # 作業位置一覧
        print("\n=== Available Positions ===")
        dobot.list_available_positions()

        print("\nOperation completed successfully!")

    except Exception as e:
        print(f"Error occurred: {str(e)}")
        exit(1)

    finally:
        if 'dobot' in locals() and dobot.api is not None:
            print("Disconnecting...")
            dobot.disconnect()
            print("Disconnected successfully")
