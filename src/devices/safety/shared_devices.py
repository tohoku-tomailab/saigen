"""
L1層: 共有デバイス管理クラス（SharedDevices）

このモジュールは複数のロボットアーム間で共有される
デバイス（カメラ、電子天秤など）を管理します。

設計思想:
- ロボットアーム（LabRobot）とは独立したライフサイクルで管理
- 複数のロボットから同じデバイスにアクセス可能
- robot_idに依存しない操作インターフェース
"""

import asyncio
import logging
import statistics
import threading
import time
from typing import Optional

from src.devices.li5640.li5640 import OPCR_AUTO_SENSITIVITY, OPCR_AUTO_SETUP

# ロギング設定
logger = logging.getLogger(__name__)


def parse_resolution(value) -> Optional[tuple]:
    """'3840x2160' / (3840, 2160) / None -> (w, h) または None（ドライバ既定）"""
    if value is None or value == "":
        return None
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return int(value[0]), int(value[1])
    text = str(value).lower().replace("\u00d7", "x").replace("*", "x")
    try:
        w, h = text.split("x")
        return int(w), int(h)
    except ValueError:
        raise ValueError(f"microscope_resolution must be 'WIDTHxHEIGHT', got {value!r}")


class SharedDevices:
    """
    共有デバイス管理クラス

    カメラや電子天秤など、特定のロボットアームに属さない
    共有リソースを管理します。
    """

    def __init__(self,
                 use_scale: bool = False,
                 use_camera: bool = False,
                 use_microscope: bool = False,
                 use_microscope_serial: bool = False,
                 use_dmm: bool = False,
                 use_lockin: bool = False,
                 use_scope: bool = False,
                 **device_configs):
        """
        SharedDevicesを初期化

        Args:
            use_scale: 電子天秤（BCE8221）を使用するか
            use_camera: Webcamを使用するか
            use_microscope: USB デジタル顕微鏡のカメラ（UVC）を使用するか
                （capture_microscope / microscope_focus）
            use_microscope_serial: 顕微鏡の制御ポート（LED・フォーカスモーター）を使用するか
                （microscope_led / microscope_focus）。microscope_port が必要
            use_dmm: Keithley 2002 デジタルマルチメータ（GPIB）を使用するか
                （measure_dmm）。dmm_resource が必要
            use_lockin: NF LI5640 ロックインアンプ（GPIB、読み取り専用）を使用するか
                （measure_lockin）。lockin_resource が必要
            use_scope: Agilent 1000 シリーズ（DSO1014A など）オシロスコープ（USB、読み取り
                専用）を使用するか（measure_scope）。scope_resource が必要
            **device_configs: デバイス固有の設定
                - scale_port: str (デフォルト: 'COM4') - BCE8221天秤のCOMポート
                - camera_index: int (デフォルト: 1) - Webcamのデバイスインデックス
                - microscope_index: int (デフォルト: 2) - 顕微鏡のデバイスインデックス
                - microscope_port: str (デフォルト: '') - 顕微鏡の制御用シリアルポート
                  (UM22 系の CP210x。空なら LED 制御なし、撮影のみ)
                - microscope_resolution: str (デフォルト: '3840x2160') - 顕微鏡の撮影解像度
                  ("幅x高さ"。データ量を抑えるなら "1280x720" など)
                - dmm_resource: str (デフォルト: '') - Keithley 2002 の VISA リソース
                  (例 'GPIB0::16::INSTR')
                - dmm_timeout_ms: int (デフォルト: 10000) - DMM の 1 回の I/O の上限
                - lockin_resource: str (デフォルト: '') - LI5640 の VISA リソース
                - lockin_timeout_ms: int (デフォルト: 5000) - LI5640 の 1 回の I/O の上限
                - wait_after_lockin: float (デフォルト: 0) - measure_lockin の後の待機秒
                  （測定の後の間隔。測定前の整定待ちではない）
                - scope_resource: str (デフォルト: '') - オシロスコープの VISA リソース
                  (USB0::0x0957::...::INSTR。シリアル番号を含む)
                - scope_timeout_ms: int (デフォルト: 5000) - オシロの 1 回の I/O の上限
                - wait_after_scope: float (デフォルト: 0) - measure_scope の後の待機秒
        """
        # デバイス使用フラグ
        self.use_scale = use_scale
        self.use_camera = use_camera
        self.use_microscope = use_microscope
        self.use_microscope_serial = use_microscope_serial
        self.use_dmm = use_dmm
        self.use_lockin = use_lockin
        self.use_scope = use_scope

        # デバイスインスタンス（初期化前はNone）
        self.scale = None
        self.camera = None
        self.microscope = None
        self.microscope_serial = None
        self.dmm = None
        self.lockin = None
        # 接続時に読んだ LI5640 の OTYP（切断時に戻す）。読む前に失敗したら None
        self._lockin_saved_otyp = None
        self.scope = None

        # DMM の I/O はワーカースレッドで行い、ロックで直列化する（測定・切断が
        # 同時にバスを触らない）。ハンドルを閉じるのは常にロックを持つ側:
        # cleanup() はロックを待ち、待ちきれなければ _dmm_closing を見た
        # ワーカーが通信を終えたところで閉じる（排他を破って閉じることはない）
        self._dmm_lock = threading.Lock()
        self._dmm_closing = threading.Event()
        # LI5640 も同じ規則（機器ごとのロック。同じ GPIB バスでも宛先は別の機器）
        self._lockin_lock = threading.Lock()
        self._lockin_closing = threading.Event()
        self._scope_lock = threading.Lock()
        self._scope_closing = threading.Event()

        # デバイス設定
        self.device_configs = device_configs

        # 待機時間設定
        self.wait_after_scale = device_configs.get('wait_after_scale', 2.0)
        self.wait_after_dmm = float(device_configs.get('wait_after_dmm') or 0.0)
        # cleanup() が DMM ワーカーの通信終了を待つ上限（秒）。超えたらワーカーに任せる
        self.dmm_close_wait_s = self._dmm_timeout_ms() / 1000.0 * 2 + 5.0
        self.wait_after_lockin = float(device_configs.get('wait_after_lockin') or 0.0)
        self.lockin_close_wait_s = self._lockin_timeout_ms() / 1000.0 * 2 + 5.0
        self.wait_after_scope = float(device_configs.get('wait_after_scope') or 0.0)
        self.scope_close_wait_s = self._scope_timeout_ms() / 1000.0 * 2 + 5.0

        # 初期化フラグ
        self._initialized = False

    async def initialize(self) -> bool:
        """
        共有デバイスを初期化

        Returns:
            bool: 初期化が成功したらTrue
        """
        try:
            logger.info("=== SharedDevices 初期化開始 ===")

            # Scale（電子天秤）の初期化
            if self.use_scale:
                if not await self._initialize_scale():
                    return False

            # Camera（カメラ）の初期化
            if self.use_camera:
                if not await self._initialize_camera():
                    return False

            # Microscope（USB デジタル顕微鏡）の初期化: カメラと制御ポートは別々に開く
            if self.use_microscope:
                if not await self._initialize_microscope():
                    return False
            if self.use_microscope_serial:
                if not await self._initialize_microscope_serial():
                    return False

            # DMM（Keithley 2002, GPIB）の初期化
            if self.use_dmm:
                if not await self._initialize_dmm():
                    return False

            # ロックインアンプ（NF LI5640, GPIB）の初期化
            if self.use_lockin:
                if not await self._initialize_lockin():
                    return False

            # オシロスコープ（Agilent 1000 シリーズ, USB）の初期化
            if self.use_scope:
                if not await self._initialize_scope():
                    return False

            self._initialized = True
            logger.info("=== SharedDevices 全デバイス初期化完了 ===\n")
            return True

        except Exception as e:
            logger.error(f"SharedDevices初期化エラー: {e}")
            return False

    async def _initialize_scale(self) -> bool:
        """BCE8221電子天秤を初期化"""
        try:
            logger.info("BCE8221電子天秤を初期化中...")
            from src.devices.scale.BCE8221 import SerialBalance

            scale_port = self.device_configs.get('scale_port', 'COM4')
            self.scale = SerialBalance(port=scale_port)

            # 接続テスト（重量を取得してみる）
            weight = self.scale.get_weight()
            if weight is None:
                logger.error("BCE8221電子天秤から重量を取得できませんでした")
                return False

            logger.info(f"✓ BCE8221電子天秤初期化完了: {weight:.3f}g")
            return True

        except Exception as e:
            logger.error(f"BCE8221電子天秤初期化エラー: {e}")
            return False

    async def _initialize_camera(self) -> bool:
        """Webcamを初期化"""
        try:
            logger.info("Webcamを初期化中...")
            from src.devices.webcam import WebcamController

            camera_index = self.device_configs.get('camera_index', 1)
            self.camera = WebcamController(camera_index=camera_index)

            # カメラ接続
            if not self.camera.connect():
                logger.error("Webcamの接続に失敗しました")
                return False

            logger.info("✓ Webcam初期化完了")
            return True

        except Exception as e:
            logger.error(f"Webcam初期化エラー: {e}")
            return False

    async def _initialize_microscope(self) -> bool:
        """USB デジタル顕微鏡のカメラ（UVC）を初期化。失敗時はハンドルを残さない"""
        try:
            logger.info("デジタル顕微鏡（カメラ）を初期化中...")
            from src.devices.microscope import MicroscopeController

            microscope_index = self.device_configs.get('microscope_index', 2)
            resolution = parse_resolution(self.device_configs.get('microscope_resolution'))
            microscope = MicroscopeController(camera_index=microscope_index, resolution=resolution)
            if not microscope.connect():
                logger.error("デジタル顕微鏡（カメラ）の接続に失敗しました")
                self._release_quietly(microscope)
                return False
            self.microscope = microscope
            logger.info("✓ デジタル顕微鏡（カメラ）初期化完了")
            return True

        except Exception as e:
            logger.error(f"デジタル顕微鏡（カメラ）初期化エラー: {e}")
            return False

    async def _initialize_microscope_serial(self) -> bool:
        """顕微鏡の制御ポート（UM22 系の CP210x シリアル）を初期化"""
        microscope_port = self.device_configs.get('microscope_port') or ""
        if not microscope_port:
            logger.error("顕微鏡の制御ポートが未設定です "
                         "(config.yaml の shared_devices.microscope_port を設定してください)")
            return False
        try:
            logger.info(f"デジタル顕微鏡（制御ポート {microscope_port}）を初期化中...")
            from src.devices.microscope import UM22SerialController

            serial_ctl = UM22SerialController(port=microscope_port)
            if not serial_ctl.connect():
                logger.error(f"デジタル顕微鏡の制御ポート {microscope_port} に接続できません")
                self._release_quietly(serial_ctl)
                return False
            self.microscope_serial = serial_ctl
            logger.info(f"✓ デジタル顕微鏡 制御ポート接続完了 ({microscope_port}, {serial_ctl.get_model()})")
            return True

        except Exception as e:
            logger.error(f"デジタル顕微鏡（制御ポート）初期化エラー: {e}")
            return False

    def _dmm_timeout_ms(self) -> int:
        return int(self.device_configs.get('dmm_timeout_ms') or 10000)

    async def _initialize_dmm(self) -> bool:
        """Keithley 2002 を開き、*IDN? で機種を確認する。失敗時はハンドルを残さない

        I/O はワーカースレッドで行う。初期化中に中断されても、開いたハンドルは
        cleanup()（またはそれを待ちきれなかった場合はワーカー自身）が閉じる。
        """
        resource = self.device_configs.get('dmm_resource') or ""
        if not resource:
            logger.error("DMM の VISA リソースが未設定です "
                         "(config.yaml の shared_devices.dmm_resource を設定してください。"
                         "例: GPIB0::16::INSTR)")
            return False
        timeout_ms = self._dmm_timeout_ms()

        def _open():
            from src.devices.keithley2002 import Keithley2002
            with self._dmm_lock:
                if self._dmm_closing.is_set():
                    raise RuntimeError("切断処理が始まっているため DMM を開きません")
                self.dmm = Keithley2002(resource, timeout_ms=timeout_ms)
                self.dmm.abort_check = self._dmm_closing.is_set
                try:
                    self.dmm.clear_status()
                    idn = self.dmm.identify()
                    self.dmm.check_errors("接続確認")
                    line_sync = self.dmm.line_sync()
                except BaseException:
                    self._release_dmm_locked()
                    raise
                finally:
                    if self._dmm_closing.is_set():
                        self._release_dmm_locked()
                return idn, line_sync

        try:
            logger.info(f"Keithley 2002（{resource}）を初期化中...")
            idn, line_sync = await asyncio.to_thread(_open)
            # ライン同期は機器全体の設定で :CONFigure では変わらない。触らずに記録する
            logger.info(f"✓ Keithley 2002 初期化完了: {idn}"
                        f"（ライン同期 {'ON' if line_sync else 'OFF'}）")
            return True
        except Exception as e:
            logger.error(f"Keithley 2002 初期化エラー（{resource}）: {e}")
            return False

    def _scope_timeout_ms(self) -> int:
        return int(self.device_configs.get('scope_timeout_ms') or 5000)

    async def _initialize_scope(self) -> bool:
        """オシロスコープを開き、機種（対応表）とエラーキューを確かめる

        送るのはデバイスクリア・``*CLS``・問い合わせだけ。失敗時はハンドルを残さない。
        """
        resource = self.device_configs.get('scope_resource') or ""
        if not resource:
            logger.error("オシロスコープの VISA リソースが未設定です "
                         "(config.yaml の shared_devices.scope_resource を設定してください。"
                         "例: USB0::0x0957::<製品ID>::<シリアル>::INSTR)")
            return False
        timeout_ms = self._scope_timeout_ms()

        def _open():
            from src.devices.dso1000 import DSO1000
            with self._scope_lock:
                if self._scope_closing.is_set():
                    raise RuntimeError("切断処理が始まっているためオシロスコープを開きません")
                self.scope = DSO1000(resource, timeout_ms=timeout_ms)
                self.scope.abort_check = self._scope_closing.is_set
                try:
                    self.scope.clear_status()
                    idn = self.scope.identify()
                    self.scope.check_errors("接続確認")
                except BaseException:
                    self._release_scope_locked()
                    raise
                finally:
                    if self._scope_closing.is_set():
                        self._release_scope_locked()
                return idn

        try:
            logger.info("オシロスコープを初期化中...")
            idn = await asyncio.to_thread(_open)
            logger.info(f"✓ オシロスコープ初期化完了: {idn}")
            return True
        except asyncio.CancelledError:
            self._scope_closing.set()       # ワーカーは次の問い合わせを送らない
            raise
        except Exception as e:
            logger.error(f"オシロスコープ初期化エラー: {e}")
            return False

    def _lockin_timeout_ms(self) -> int:
        return int(self.device_configs.get('lockin_timeout_ms') or 5000)

    async def _initialize_lockin(self) -> bool:
        """LI5640 を開き、機種・エラーキュー・連続出力の停止を確かめ、OTYP を覚える

        失敗時はハンドルを残さない（OTYP を読む前なら復元もしない）。I/O は
        ワーカースレッドで行い、中断されても cleanup()（またはワーカー自身）が閉じる。
        """
        resource = self.device_configs.get('lockin_resource') or ""
        if not resource:
            logger.error("ロックインアンプの VISA リソースが未設定です "
                         "(config.yaml の shared_devices.lockin_resource を設定してください。"
                         "例: GPIB0::<アドレス>::INSTR)")
            return False
        timeout_ms = self._lockin_timeout_ms()

        def _open():
            from src.devices.li5640 import LI5640, LI5640Error
            with self._lockin_lock:
                if self._lockin_closing.is_set():
                    raise RuntimeError("切断処理が始まっているため LI5640 を開きません")
                self.lockin = LI5640(resource, timeout_ms=timeout_ms)
                self.lockin.abort_check = self._lockin_closing.is_set
                try:
                    self.lockin.clear_status()
                    idn = self.lockin.identify()
                    self.lockin.check_errors("接続確認")
                    if self.lockin.continuous_output():
                        raise LI5640Error("連続データ出力（OSTR 1）が動いています。"
                                          "前面パネルの操作か別のソフトで止めてから実行してください"
                                          "（SAIGEN は止めるコマンドを送りません）")
                    self._lockin_saved_otyp = self.lockin.output_type()
                except BaseException:
                    self._release_lockin_locked()
                    raise
                finally:
                    if self._lockin_closing.is_set():
                        self._release_lockin_locked()
                return idn, self._lockin_saved_otyp

        try:
            logger.info(f"LI5640（{resource}）を初期化中...")
            idn, otyp = await asyncio.to_thread(_open)
            logger.info(f"✓ LI5640 初期化完了: {idn}（OTYP {otyp} を切断時に戻します）")
            return True
        except asyncio.CancelledError:
            # 中断されたら、ワーカーは次の問い合わせを送らない（切断は cleanup() が行う）
            self._lockin_closing.set()
            raise
        except Exception as e:
            logger.error(f"LI5640 初期化エラー（{resource}）: {e}")
            return False

    @staticmethod
    def _release_quietly(device) -> None:
        """初期化に失敗したデバイスのハンドルをベストエフォートで閉じる"""
        for name in ("disconnect", "close"):
            fn = getattr(device, name, None)
            if callable(fn):
                try:
                    fn()
                except Exception:
                    pass
                return

    # ========================================
    # 電子天秤操作
    # ========================================

    async def measure_weight(self, stabilization_count: int = 3) -> float:
        """
        BCE8221で安定した重量測定を行う

        Args:
            stabilization_count: 測定回数（中央値を返す）

        Returns:
            float: 測定重量（g）

        Raises:
            RuntimeError: 電子天秤が初期化されていない場合
            ValueError: 有効な重量が取得できなかった場合
        """
        if not self.use_scale or self.scale is None:
            raise RuntimeError("BCE8221電子天秤が初期化されていません")

        try:
            weights = []
            for i in range(stabilization_count):
                weight = self.scale.get_weight()
                if weight is not None:
                    weights.append(weight)
                await asyncio.sleep(0.5)

            if not weights:
                raise ValueError("有効な重量データを取得できませんでした")

            # 中央値を返す（外れ値の影響を減らす）
            weights.sort()
            median_weight = weights[len(weights) // 2]

            logger.info(f"✓ 重量測定: {median_weight:.3f}g")
            await asyncio.sleep(self.wait_after_scale)
            return median_weight

        except Exception as e:
            logger.error(f"重量測定エラー: {e}")
            raise

    async def tare_scale(self, delay: float = 1.0):
        """
        BCE8221電子天秤を風袋引き（ゼロ点リセット）

        Args:
            delay: 風袋引き後の待機時間（秒）

        Raises:
            RuntimeError: 電子天秤が初期化されていない場合
        """
        if not self.use_scale or self.scale is None:
            raise RuntimeError("BCE8221電子天秤が初期化されていません")

        try:
            logger.info("BCE8221電子天秤を風袋引き中...")
            self.scale.tare()
            await asyncio.sleep(delay)
            logger.info("✓ 風袋引き完了")

        except Exception as e:
            logger.error(f"風袋引きエラー: {e}")
            raise

    # ========================================
    # DMM（Keithley 2002）操作
    # ========================================

    async def measure_dmm(self, function: str = "dc_voltage", samples: int = 3,
                          nplc: float = 1.0, range_: Optional[float] = None) -> dict:
        """
        Keithley 2002 で測定する（samples 回測って中央値）

        Args:
            function: 測定機能（dc_voltage / ac_voltage / dc_current / ac_current /
                resistance_2w / resistance_4w）
            samples: 測定回数。有効値の中央値を返す（偶数個なら中央 2 値の平均）
            nplc: 積分時間（電源周期の倍数、0.01〜50）
            range_: 固定レンジ（その機能の単位）。None ならオートレンジ

        Returns:
            dict: ``dmm_value``（中央値）, ``dmm_unit``, ``dmm_function``,
            ``dmm_range``（機器から読み返したレンジ）, ``dmm_nplc``,
            ``dmm_valid_samples``（有効だった回数）

        Raises:
            RuntimeError: DMM が初期化されていない場合
            ValueError: 有効な測定値が 1 つも得られなかった場合
            Keithley2002Error: 機器が設定を受け付けなかった場合

        ``dmm_range`` は最後の読み取りの後に読み返したレンジ（オートレンジでは
        最後のサンプルで使われたレンジ）。

        中断（Ctrl+C / GUI の Stop）では読み取り中のワーカーを待たずに再送出する
        （session の緊急停止を遅らせない）。ワーカーは進行中の 1 回の I/O
        （上限 dmm_timeout_ms）を終えたところで残りを打ち切り、cleanup() は
        ロックを取ってそれを待ってから GTL・切断する。
        """
        if not self.use_dmm or self.dmm is None or self._dmm_closing.is_set():
            raise RuntimeError("Keithley 2002 が初期化されていません")
        dmm = self.dmm
        stop = threading.Event()        # この測定だけの打ち切り要求

        def _run():
            from src.devices.keithley2002 import Keithley2002Aborted
            with self._dmm_lock:
                if self.dmm is not dmm or self._dmm_closing.is_set():
                    return None, [], True
                # 打ち切り要求は write / query の直前に見る（configure の途中、
                # 最後の読み取りの後の読み返しの前も含め、すべての I/O 境界）
                dmm.abort_check = lambda: stop.is_set() or self._dmm_closing.is_set()
                try:
                    settings = dmm.configure(function, range_=range_, nplc=nplc)
                    values = []
                    for _ in range(int(samples)):
                        value = dmm.read()
                        if value is not None:
                            values.append(value)
                    settings.update(dmm.read_settings(function))
                    return settings, values, False
                except Keithley2002Aborted:
                    return None, [], True
                finally:
                    dmm.abort_check = self._dmm_closing.is_set
                    if self._dmm_closing.is_set():
                        self._release_dmm_locked()

        try:
            settings, values, stopped = await asyncio.to_thread(_run)
        except asyncio.CancelledError:
            stop.set()
            raise
        except Exception as e:
            logger.error(f"DMM 測定エラー: {e}")
            raise
        if stopped:
            raise RuntimeError("DMM 測定が中断されました")
        if not values:
            raise ValueError(f"DMM から有効な測定値を取得できませんでした（{samples} 回）")

        median = statistics.median(values)
        unit = settings["unit"]
        logger.info(f"✓ DMM 測定 {function}: {median!r} {unit}"
                    f"（有効 {len(values)}/{samples} 回, range {settings.get('range')}, "
                    f"NPLC {settings.get('nplc')}）")
        if self.wait_after_dmm > 0:
            await asyncio.sleep(self.wait_after_dmm)
        return {
            "dmm_value": median,
            "dmm_unit": unit,
            "dmm_function": function,
            "dmm_range": settings.get("range"),
            "dmm_nplc": settings.get("nplc"),
            "dmm_valid_samples": len(values),
        }

    def _release_dmm_locked(self) -> bool:
        """GTL で前面操作に戻して閉じる。呼び出し側が _dmm_lock を持っていること

        送出しない（cleanup() から委ねられたワーカーが呼ぶと、例外の受け手がいない）。
        close() に失敗したらエラーをログに残し、ハンドルを ``self.dmm`` に残す
        （次の cleanup() で再試行できる）。

        Returns:
            bool: 閉じた（または開いていなかった）なら True
        """
        dmm = self.dmm
        if dmm is None:
            return True
        try:
            dmm.local()
        except Exception as e:  # noqa: BLE001 - 閉じることを優先する
            logger.warning(f"Keithley 2002 を前面操作に戻せません（GTL）: {e}")
        try:
            dmm.close()
        except Exception as e:  # noqa: BLE001 - 記録してハンドルを残す
            logger.error(f"Keithley 2002 を閉じられません（次の cleanup() で再試行します）: {e}")
            return False
        self.dmm = None
        logger.info("✓ Keithley 2002 切断完了")
        return True

    async def _close_dmm(self):
        """DMM を閉じる。通信中のワーカーがいればその I/O が終わるのを待つ

        待ちきれない場合もハンドルには触らない（通信中に閉じない）。
        _dmm_closing を見たワーカーが、通信を終えた時点で GTL・切断する。
        """
        self._dmm_closing.set()

        def _close():
            if not self._dmm_lock.acquire(timeout=self.dmm_close_wait_s):
                logger.warning("DMM の通信が終わっていません。通信を終えたワーカーが GTL・切断します")
                return True
            try:
                return self._release_dmm_locked()
            finally:
                self._dmm_lock.release()

        if not await asyncio.to_thread(_close):
            raise RuntimeError("Keithley 2002 を閉じられませんでした（ハンドルは保持しています）")

    # ========================================
    # ロックインアンプ（NF LI5640）操作
    # ========================================

    async def measure_lockin(self, samples: int = 5, interval: float = 0.0) -> dict:
        """
        LI5640 のパネル表示値（DATA1 / DATA2）と参照周波数を samples 回読み、中央値を返す

        測定設定は変えない（送る設定は OTYP だけ。切断時に戻す）。読み取りの前後で
        パネル設定を読み、違っていればステップを失敗にする（途中で変えて戻された
        場合は検出できない）。値の意味（X / R / θ ...、単位）はその設定から決める。

        Args:
            samples: 読み取り回数（1〜100）
            interval: 読み取りの間隔（秒）。整定待ちではない（測定前の整定は人が待つ）

        Returns:
            dict: ``lockin_data1`` / ``lockin_data2``（有効サンプルの中央値。θ は円弧上の
            中央値）, ``lockin_data{1,2}_kind`` / ``_unit``, ``lockin_freq``（Hz）,
            ``lockin_sensitivity``, ``lockin_time_constant``, ``lockin_valid_samples``,
            ``lockin_invalid``（捨てた理由ごとの数）, ``lockin_settings``（設定の dict）

        Raises:
            RuntimeError: 初期化されていない・中断された
            LockinStepError: 設定の変化、有効サンプルなし、θ が曖昧（``diagnostics`` 付き）
            LI5640Error: 機器のエラー・解釈できない設定値。I/O エラーはそのまま送出
        """
        from src.devices.li5640 import LockinStepError
        from src.devices.li5640.li5640 import (describe, format_si, invalid_reason,
                                               sensitivity_value, snapshot_diff, theta_median,
                                               time_constant_value)
        if not self.use_lockin or self.lockin is None or self._lockin_closing.is_set():
            raise RuntimeError("LI5640 が初期化されていません")
        samples = int(samples)
        interval = float(interval)
        if samples < 1:
            raise ValueError(f"samples は 1 以上: {samples}")
        lia = self.lockin
        stop = threading.Event()        # この測定だけの打ち切り要求

        def _stopped() -> bool:
            return stop.is_set() or self._lockin_closing.is_set()

        def _run():
            from src.devices.li5640 import READ_OUTPUT_TYPE, LI5640Aborted
            with self._lockin_lock:
                if self.lockin is not lia or self._lockin_closing.is_set():
                    return None
                lia.abort_check = _stopped
                try:
                    lia.set_output_type(READ_OUTPUT_TYPE)
                    before = lia.snapshot()
                    reads = []
                    for i in range(samples):
                        if i and interval > 0:
                            # 中断・切断要求を 0.1 秒ごとに見ながら待つ
                            deadline = time.monotonic() + interval
                            while (left := deadline - time.monotonic()) > 0:
                                if stop.wait(min(0.1, left)) or _stopped():
                                    return None
                        reads.append(lia.read_sample())
                    after = lia.snapshot()
                    return before, after, reads
                except LI5640Aborted:
                    return None
                finally:
                    lia.abort_check = self._lockin_closing.is_set
                    if self._lockin_closing.is_set():
                        self._release_lockin_locked()

        try:
            outcome = await asyncio.to_thread(_run)
        except asyncio.CancelledError:
            stop.set()
            raise
        except Exception as e:
            logger.error(f"LI5640 測定エラー: {e}")
            raise
        if outcome is None:
            raise RuntimeError("LI5640 の測定が中断されました")
        before, after, reads = outcome

        # サンプルの分類を先に済ませ、どの失敗の診断にも捨てた理由の件数を入れる
        desc = describe(before)
        valid = []
        invalid = {}
        for sample, warning, _raw in reads:
            reason = invalid_reason(sample, warning, desc)
            if reason is None:
                valid.append(sample)
            else:
                invalid[reason] = invalid.get(reason, 0) + 1
        diagnostics = {"settings_before": before, "settings_after": after,
                       "samples": samples, "interval_s": interval,
                       "invalid": invalid, "display": desc}
        diff = snapshot_diff(before, after)
        if diff:
            diagnostics["settings_diff"] = diff
            logger.error(f"LI5640 の設定が測定中に変わりました: {diff}")
            raise LockinStepError(f"LI5640 の設定が測定中に変わりました: {sorted(diff)}", diagnostics)
        if before["opcr"] & (OPCR_AUTO_SETUP | OPCR_AUTO_SENSITIVITY):
            raise LockinStepError(f"LI5640 で自動設定が実行中です（OPCR {before['opcr']}）",
                                  diagnostics)
        if invalid:
            logger.warning(f"LI5640: {sum(invalid.values())}/{samples} 回のサンプルを捨てました {invalid}")
        if not valid:
            raise LockinStepError(f"LI5640 から有効なサンプルを取得できませんでした"
                                  f"（{samples} 回: {invalid}）", diagnostics)

        data1 = statistics.median(s["data1"] for s in valid)
        if desc["data2_kind"] == "THETA":
            data2 = theta_median([s["data2"] for s in valid])
            if data2 is None:
                raise LockinStepError("LI5640 の位相が幅 180° 以上に散らばっていて、"
                                      "中央値を決められません", diagnostics)
        else:
            data2 = statistics.median(s["data2"] for s in valid)
        freq = statistics.median(s["freq"] for s in valid)

        current = before["isrc"] in (2, 3)
        sens_code = before["isen"] if current else before["vsen"]
        sens_unit = ("A" if current else "V") + ("/√Hz" if desc["data1_kind"] == "NOISE" else "")
        sensitivity = format_si(sensitivity_value(sens_code, current), sens_unit)
        time_constant = format_si(time_constant_value(before["tcon"]), "s")

        def kind(n):
            mode = desc[f"data{n}_mode"]
            return desc[f"data{n}_kind"] + ("" if mode == "plain" else f"[{mode}]")

        logger.info(f"✓ LI5640: DATA1 {kind(1)} = {data1!r} {desc['data1_unit']}, "
                    f"DATA2 {kind(2)} = {data2!r} {desc['data2_unit']}, f = {freq!r} Hz"
                    f"（有効 {len(valid)}/{samples} 回, 感度 {sensitivity}, 時定数 {time_constant}）")
        if self.wait_after_lockin > 0:
            await asyncio.sleep(self.wait_after_lockin)
        return {
            "lockin_data1": data1,
            "lockin_data1_kind": kind(1),
            "lockin_data1_unit": desc["data1_unit"],
            "lockin_data2": data2,
            "lockin_data2_kind": kind(2),
            "lockin_data2_unit": desc["data2_unit"],
            "lockin_freq": freq,
            "lockin_sensitivity": sensitivity,
            "lockin_time_constant": time_constant,
            "lockin_valid_samples": len(valid),
            "lockin_invalid": invalid,
            "lockin_settings": before,
        }

    def _release_lockin_locked(self) -> bool:
        """クリア・OTYP 復元・GTL・切断。呼び出し側が _lockin_lock を持っていること

        送出しない。close() に失敗したらエラーをログに残し、ハンドルを残す
        （次の cleanup() で再試行できる。そのときも OTYP の復元から試す）。
        """
        lia = self.lockin
        if lia is None:
            return True
        try:
            lia.release(self._lockin_saved_otyp)
        except Exception as e:  # noqa: BLE001 - 記録してハンドルを残す
            logger.error(f"LI5640 を閉じられません（次の cleanup() で再試行します）: {e}")
            return False
        self.lockin = None
        self._lockin_saved_otyp = None
        logger.info("✓ LI5640 切断完了")
        return True

    async def _close_lockin(self):
        """LI5640 を閉じる。通信中のワーカーがいればその I/O が終わるのを待つ（DMM と同じ）"""
        self._lockin_closing.set()

        def _close():
            if not self._lockin_lock.acquire(timeout=self.lockin_close_wait_s):
                logger.warning("LI5640 の通信が終わっていません。通信を終えたワーカーが"
                               "OTYP 復元・GTL・切断します")
                return True
            try:
                return self._release_lockin_locked()
            finally:
                self._lockin_lock.release()

        if not await asyncio.to_thread(_close):
            raise RuntimeError("LI5640 を閉じられませんでした（ハンドルは保持しています）")

    # ========================================
    # オシロスコープ（Agilent 1000 シリーズ）操作
    # ========================================

    async def measure_scope(self, channel: int = 1, items=("vpp", "frequency"),
                            samples: int = 3, interval: float = 0.5) -> dict:
        """
        オシロスコープの自動測定値を読む（問い合わせだけ。設定は前面パネルのもの）

        ``samples`` 回、要求された項目を順に ``:MEASure:<項目>? CHANnel<n>`` で読み、
        項目ごとに有効な値の中央値を返す。``samples`` は読み取り回数で、新しい取り込み
        の回数でも独立な標本数でもない。1 回の読み取りの中の項目は同時ではない。

        読み取りの前後で設定を読み、違えば失敗（途中で変えて戻された場合は検出できない）。
        時間軸が X-Y、時間軸モードが DELAYED、トリガがエッジ以外、チャネル非表示も失敗。
        要求された項目のどれか 1 つでも有効な値が 0 回なら失敗。I/O エラー・読めない設定・
        機器のエラーキューの項目も失敗（有効な値がほかにあっても）。

        Returns:
            dict: ``scope_channel``, ``scope_values`` / ``scope_units`` / ``scope_valid``
            （項目ごと）, ``scope_invalid``（項目ごと・理由ごとの件数）, ``scope_settings``,
            ``scope_settings_after``（読み取り後の設定。状態・サンプルレート・
            :MEASure:SOURce の変化も残る）, ``scope_raw``（項目ごとの応答文字列の列）

        Raises:
            RuntimeError: 初期化されていない・中断された
            ValueError: 引数が範囲外
            ScopeStepError: 上記の失敗（``diagnostics`` 付き）
            DSO1000Error: 機器のエラー・解釈できない設定値。I/O エラーはそのまま送出
        """
        from statistics import median

        from src.devices.dso1000 import ITEMS, ScopeStepError
        from src.devices.dso1000.dso1000 import VOLATILE_SETTINGS, item_unit, parse_measurement
        if not self.use_scope or self.scope is None or self._scope_closing.is_set():
            raise RuntimeError("オシロスコープが初期化されていません")
        channel, samples, interval = int(channel), int(samples), float(interval)
        items = list(items)
        if not items or len(set(items)) != len(items) or any(i not in ITEMS for i in items):
            raise ValueError(f"items は重複のない既知の項目 1 つ以上: {items}")
        if samples < 1:
            raise ValueError(f"samples は 1 以上: {samples}")
        dso = self.scope
        if not 1 <= channel <= (dso.channels or 4):
            raise ScopeStepError(f"{dso.model} は CH{channel} を持ちません（1〜{dso.channels}）",
                                 {"channel": channel, "model": dso.model})
        stop = threading.Event()

        def _stopped() -> bool:
            return stop.is_set() or self._scope_closing.is_set()

        def _run():
            from src.devices.dso1000 import DSO1000Aborted
            with self._scope_lock:
                if self.scope is not dso or self._scope_closing.is_set():
                    return None
                dso.abort_check = _stopped
                progress = {"channel": channel}     # 失敗時の診断用（取得済みのものだけ）
                try:
                    fmt = dso.timebase_format()
                    if fmt == "XY":
                        return "reject", {"tb_format": fmt}, "時間軸が X-Y 表示です（自動測定できません）"
                    before = dso.snapshot(channel, into=progress.setdefault("settings_before", {}))
                    if before["tb_mode"] != "MAIN":
                        return "reject", before, "時間軸が DELAYED（ズーム）です（MAIN だけに対応）"
                    if before["trig_mode"] != "EDGE":
                        return "reject", before, (f"トリガが {before['trig_mode']} です"
                                                  "（エッジトリガだけに対応）")
                    if not before["ch_display"]:
                        return "reject", before, f"CH{channel} が表示されていません"
                    raw = {item: [] for item in items}
                    progress["raw"] = raw
                    for i in range(samples):
                        if i and interval > 0:
                            deadline = time.monotonic() + interval
                            while (left := deadline - time.monotonic()) > 0:
                                if stop.wait(min(0.1, left)) or _stopped():
                                    return None
                        for item in items:
                            raw[item].append(dso.measure(item, channel))
                    after = dso.snapshot(channel, into=progress.setdefault("settings_after", {}))
                    dso.check_errors("測定の後", into=progress.setdefault("errors_after", []))
                    return "ok", before, after, raw
                except DSO1000Aborted:
                    return None
                except Exception as e:
                    # I/O エラー・機器エラーでも、取得済みの設定と応答を記録に残す
                    # （追加の I/O はしない）。例外の型はそのまま
                    try:
                        e.diagnostics = {**progress, "error_type": type(e).__name__}
                    except Exception:  # noqa: BLE001 - 属性を持てない例外はそのまま
                        pass
                    raise
                finally:
                    dso.abort_check = self._scope_closing.is_set
                    if self._scope_closing.is_set():
                        self._release_scope_locked()

        try:
            outcome = await asyncio.to_thread(_run)
        except asyncio.CancelledError:
            stop.set()
            raise
        except Exception as e:
            logger.error(f"オシロスコープ測定エラー: {e}")
            raise
        if outcome is None:
            raise RuntimeError("オシロスコープの測定が中断されました")
        if outcome[0] == "reject":
            _, settings, reason = outcome
            raise ScopeStepError(reason, {"channel": channel, "settings_before": settings})
        _, before, after, raw = outcome

        values, units, valid, invalid = {}, {}, {}, {}
        for item in items:
            good = []
            for reply in raw[item]:
                value, reason = parse_measurement(reply, item)
                if reason is None:
                    good.append(value)
                else:
                    invalid.setdefault(item, {})
                    invalid[item][reason] = invalid[item].get(reason, 0) + 1
            values[item] = median(good) if good else None
            units[item] = item_unit(item, before["ch_units"])
            valid[item] = len(good)
        diagnostics = {"channel": channel, "settings_before": before, "settings_after": after,
                       "samples": samples, "interval_s": interval, "values": values,
                       "valid": valid, "invalid": invalid, "raw": raw}
        compare = [k for k in set(before) | set(after) if k not in VOLATILE_SETTINGS]
        diff = {k: [before.get(k), after.get(k)] for k in sorted(compare)
                if before.get(k) != after.get(k)}
        if diff:
            diagnostics["settings_diff"] = diff
            logger.error(f"オシロスコープの設定が測定中に変わりました: {diff}")
            raise ScopeStepError(f"オシロスコープの設定が測定中に変わりました: {sorted(diff)}",
                                 diagnostics)
        empty = [item for item in items if valid[item] == 0]
        if empty:
            raise ScopeStepError(f"有効な値を取得できなかった項目があります: {empty}"
                                 f"（{ {i: invalid.get(i) for i in empty} }）", diagnostics)
        if invalid:
            logger.warning(f"オシロスコープ: 捨てた読み取り {invalid}")
        if before["status"] == "STOP" or after["status"] == "STOP":
            logger.warning("オシロスコープが STOP 状態でした（同じ取り込みを読んでいる可能性があります）")
        shown = ", ".join(f"{i} = {values[i]!r} {units[i]}" for i in items)
        logger.info(f"✓ オシロスコープ CH{channel}: {shown}（{samples} 回読み取り, "
                    f"{before['ch_scale']:g} /div, {before['tb_scale']:g} s/div, "
                    f"状態 {before['status']}）")
        if self.wait_after_scope > 0:
            await asyncio.sleep(self.wait_after_scope)
        return {"scope_channel": channel, "scope_values": values, "scope_units": units,
                "scope_valid": valid, "scope_invalid": invalid, "scope_settings": before,
                "scope_settings_after": after, "scope_raw": raw}

    def _release_scope_locked(self) -> bool:
        """デバイスクリア・切断。呼び出し側が _scope_lock を持っていること。送出しない"""
        dso = self.scope
        if dso is None:
            return True
        try:
            dso.release()
        except Exception as e:  # noqa: BLE001 - 記録してハンドルを残す
            logger.error(f"オシロスコープを閉じられません（次の cleanup() で再試行します）: {e}")
            return False
        self.scope = None
        logger.info("✓ オシロスコープ切断完了")
        return True

    async def _close_scope(self):
        """オシロスコープを閉じる。通信中のワーカーがいればその I/O が終わるのを待つ"""
        self._scope_closing.set()

        def _close():
            if not self._scope_lock.acquire(timeout=self.scope_close_wait_s):
                logger.warning("オシロスコープの通信が終わっていません。通信を終えたワーカーが切断します")
                return True
            try:
                return self._release_scope_locked()
            finally:
                self._scope_lock.release()

        if not await asyncio.to_thread(_close):
            raise RuntimeError("オシロスコープを閉じられませんでした（ハンドルは保持しています）")

    # ========================================
    # カメラ操作
    # ========================================

    async def capture_and_save(self, file_path: Optional[str] = None) -> Optional[str]:
        """
        Webcamで画像をキャプチャして保存

        Args:
            file_path: 保存先のファイル名（省略時は自動生成）

        Returns:
            Optional[str]: 保存したファイルパス。失敗時はNone

        Raises:
            RuntimeError: カメラが初期化されていません
        """
        if not self.use_camera or self.camera is None:
            raise RuntimeError("カメラが初期化されていません")

        try:
            logger.info("Webcamで画像キャプチャ・保存中...")
            saved_path = self.camera.capture_and_save(file_path)

            if saved_path:
                logger.info(f"✓ 画像キャプチャ・保存完了: {saved_path}")
            else:
                logger.warning("画像の取得・保存に失敗しました")

            return saved_path

        except Exception as e:
            logger.error(f"画像キャプチャ・保存エラー: {e}")
            raise

    # ========================================
    # デジタル顕微鏡操作
    # ========================================

    async def capture_microscope(self, file_path: Optional[str] = None) -> Optional[str]:
        """
        USB デジタル顕微鏡で画像をキャプチャして保存

        Args:
            file_path: 保存先のファイル名（省略時は自動生成、microscope_images/ 配下）

        Returns:
            Optional[str]: 保存したファイルパス。失敗時はNone

        Raises:
            RuntimeError: 顕微鏡が初期化されていない場合
        """
        if not self.use_microscope or self.microscope is None:
            raise RuntimeError("デジタル顕微鏡が初期化されていません")

        try:
            logger.info("デジタル顕微鏡で画像キャプチャ・保存中...")
            saved_path = self.microscope.capture_and_save(file_path)

            if saved_path:
                logger.info(f"✓ 顕微鏡画像キャプチャ・保存完了: {saved_path}")
            else:
                logger.warning("顕微鏡画像の取得・保存に失敗しました")

            return saved_path

        except Exception as e:
            logger.error(f"顕微鏡画像キャプチャ・保存エラー: {e}")
            raise

    async def set_microscope_led(self, on: bool = True, level: Optional[int] = None) -> bool:
        """
        USB デジタル顕微鏡の LED 照明を点灯/消灯し、必要なら明るさも設定する

        Args:
            on: True = 点灯, False = 消灯
            level: 明るさ (0-255、出荷時 12)。None なら変更しない

        Returns:
            bool: 実行後の LED 状態

        Raises:
            RuntimeError: 顕微鏡の制御ポート (microscope_port) が設定・接続されていない場合
        """
        if not self.use_microscope_serial or self.microscope_serial is None:
            raise RuntimeError(
                "顕微鏡の制御ポートが初期化されていません "
                "(config.yaml の shared_devices.microscope_port を設定してください)"
            )
        try:
            state = self.microscope_serial.set_led(on, level)
            await asyncio.sleep(0.2)
            return state
        except Exception as e:
            logger.error(f"顕微鏡 LED 制御エラー: {e}")
            raise

    async def focus_microscope(self, mode: str = "auto", position: Optional[int] = None,
                               direction: str = "in", steps: int = 1,
                               timeout: float = 60.0) -> dict:
        """
        USB デジタル顕微鏡（UM22 系）の焦点合わせ

        Args:
            mode: "auto" = ワンショット AF / "position" = レンズ位置を直接指定 /
                  "step" = ステップ移動
            position: mode="position" のときの目標位置 (0-65535)
            direction: mode="step" のときの方向 ("in" / "out")
            steps: mode="step" のときの回数
            timeout: モーター停止を待つ上限秒

        Returns:
            dict: {"focus_position": 最終レンズ位置, "focus_converged": bool}

        Raises:
            RuntimeError: 顕微鏡の制御ポートが初期化されていない場合
        """
        if not self.use_microscope_serial or self.microscope_serial is None:
            raise RuntimeError(
                "顕微鏡の制御ポートが初期化されていません "
                "(config.yaml の shared_devices.microscope_port を設定してください)"
            )
        if mode not in ("auto", "position", "step"):
            raise ValueError(f"unknown focus mode: {mode!r}")
        if mode == "position" and position is None:
            raise ValueError("mode='position' には position が必要です")
        if mode == "auto" and self.microscope is None:
            raise RuntimeError("AF には顕微鏡カメラの映像が必要です（use_microscope=True で初期化してください）")
        scope = self.microscope_serial

        def _run():
            if mode == "auto":
                return scope.autofocus(timeout_s=timeout)
            if mode == "position":
                return scope.goto_position(position, timeout_s=timeout)
            return scope.step_focus(direction, steps, timeout_s=timeout)

        # AF の評価値は映像パイプラインが動いているときだけ更新されるので、
        # モーターが動いている間はカメラからフレームを読み続ける。
        # キャンセル（Ctrl+C 等）時はワーカーに停止を要求し、モーターが止まって
        # ワーカーが終わるまで待ってからポンプを止める（ポートを閉じる前に必ず終える）。
        with self._pump_microscope_frames():
            worker = asyncio.ensure_future(asyncio.to_thread(_run))
            try:
                pos, ok = await asyncio.shield(worker)
            except asyncio.CancelledError:
                scope.request_stop()
                try:
                    await asyncio.wait_for(asyncio.shield(worker), timeout=max(15.0, timeout))
                except Exception:
                    pass
                raise
            except Exception as e:
                logger.error(f"顕微鏡フォーカス制御エラー: {e}")
                raise
        await asyncio.sleep(0.2)
        return {"focus_position": pos, "focus_converged": bool(ok)}

    def _pump_microscope_frames(self, interval_s: float = 0.1):
        """顕微鏡カメラのフレームを背景スレッドで読み続けるコンテキストマネージャ"""
        shared = self

        class _Pump:
            def __enter__(self_):
                self_.stop = threading.Event()
                self_.frames = 0
                cam = getattr(shared.microscope, "camera", None)
                self_.thread = None
                if cam is None or not hasattr(cam, "read"):
                    return self_

                def loop():
                    while not self_.stop.is_set():
                        try:
                            ok, _ = cam.read()
                            if ok:
                                self_.frames += 1
                        except Exception:
                            pass
                        self_.stop.wait(interval_s)

                self_.thread = threading.Thread(target=loop, name="microscope-frame-pump", daemon=True)
                self_.thread.start()
                return self_

            def __exit__(self_, *exc):
                self_.stop.set()
                if self_.thread is not None:
                    self_.thread.join(timeout=10.0)
                    if self_.thread.is_alive():
                        logger.warning("顕微鏡フレームポンプのスレッドが 10 秒以内に終了しません "
                                       "(カメラの read() がブロックしています)")
                logger.debug(f"顕微鏡フレームポンプ終了: {self_.frames} frames")
                return False

        return _Pump()

    # ========================================
    # ライフサイクル管理
    # ========================================

    async def cleanup(self):
        """全共有デバイスを安全に切断"""
        logger.info("=== SharedDevices 切断開始 ===")

        try:
            if self.scale:
                self.scale.close()
                logger.info("✓ BCE8221電子天秤切断完了")
        except Exception as e:
            logger.error(f"BCE8221電子天秤切断エラー: {e}")

        try:
            if self.camera:
                self.camera.disconnect()
                logger.info("✓ Webcam切断完了")
        except Exception as e:
            logger.error(f"Webcam切断エラー: {e}")

        try:
            if self.microscope:
                self.microscope.disconnect()
                logger.info("✓ デジタル顕微鏡切断完了")
        except Exception as e:
            logger.error(f"デジタル顕微鏡切断エラー: {e}")

        try:
            if self.microscope_serial:
                self.microscope_serial.disconnect()
                logger.info("✓ デジタル顕微鏡 制御ポート切断完了")
        except Exception as e:
            logger.error(f"デジタル顕微鏡 制御ポート切断エラー: {e}")

        # GPIB 機器: self.dmm ではなく use_dmm で判定する（初期化ワーカーがまだ動いていても
        # _close_* はロックを取ってから開いたハンドルを閉じる）。切断を待っている間に
        # 中断されても残りの機器の切断を続け、全部試してから中断を送出する
        # 先に全機器の切断要求を立てる: 1 台目の切断を待っている間に、2 台目の
        # ワーカー（初期化中・測定中）が通常の問い合わせを送り続けないように
        if self.use_dmm:
            self._dmm_closing.set()
        if self.use_lockin:
            self._lockin_closing.set()
        if self.use_scope:
            self._scope_closing.set()
        cancelled = None
        for label, used, close in (("Keithley 2002", self.use_dmm, self._close_dmm),
                                   ("LI5640", self.use_lockin, self._close_lockin),
                                   ("オシロスコープ", self.use_scope, self._close_scope)):
            if not used:
                continue
            try:
                await close()
            except asyncio.CancelledError as e:
                logger.error(f"{label} の切断中に中断されました。残りの機器の切断を続けます")
                cancelled = e
            except Exception as e:
                logger.error(f"{label} 切断エラー: {e}")
        if cancelled is not None:
            raise cancelled

        logger.info("=== SharedDevices 全デバイス切断完了 ===")

    async def __aenter__(self):
        """非同期コンテキストマネージャ: 入口"""
        if not await self.initialize():
            raise RuntimeError("SharedDevicesの初期化に失敗しました")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """非同期コンテキストマネージャ: 出口"""
        await self.cleanup()
