"""
L1層: 中断可能なブロッキング・ドライバ呼び出し（InterruptibleDriverMixin）

LabRobot（lab_robot.py の _run_blocking / _wait_worker / _read_pose）と同じ
安全動作を、アームの種類に依存しない形で切り出したもの。具体的なクラスは
次のフックと属性を用意する:

フック:
    _force_stop_arm() -> bool
        アームをその場で止める（ベストエフォート、例外を送出しない）。
        停止コマンドが受理されたら True。
    _driver_pose() -> list[float]
        アームから現在位置を読む（ブロッキング）。
    _pose_indices(cur, prev) -> list[int]
        静止判定で比較する要素の添字（既定は [0, 1, 2] = X, Y, Z）。
    _driver_reports_idle() -> bool
        直前の _driver_pose() でドライバが「動作中ではない」と報告したか
        （既定は True。動作中フラグを持つドライバは上書きする）。
    _on_pose_reread(pose)
        静止を確認して読み直した位置を受け取る（既定は何もしない）。

属性:
    _pose_unknown: bool       移動が中断・失敗した後は True
    _estop_generation: int    emergency_stop() のたびに増やす
    pose_settle_reads: int    静止確認の最大読み取り回数
    pose_settle_interval: float  読み取り間隔（秒）
    pose_settle_tolerance: float 静止とみなす変化量の上限
    stop_wait_timeout: float  強制停止後にワーカーの終了を待つ上限（秒）
    _arm_name: str            ログ用の名前

緊急停止の世代（_estop_generation）:
    公開の移動操作（move_xyz など）は入口で ``gen = self._estop_generation`` を取り、
    その操作の全区間（_read_pose(gen) の読み直し、ラッチ解除、_run_blocking(...,
    estop_before=gen) の移動）に渡す。途中で世代が変わったら（別スレッドから
    emergency_stop() された）、姿勢不明のまま RuntimeError（「緊急停止」）を送出し、
    それ以上コマンドを送らない。``_pose_unknown = False`` を書いた直後にも世代を
    確認し直す（確認と書き込みの間に割り込んだ緊急停止を取りこぼさない）。
    emergency_stop() 側は「世代を増やす → _pose_unknown = True を書く」の順にすること。

放棄したワーカー:
    強制停止後 stop_wait_timeout 秒待っても戻らないワーカースレッドは残して中断を
    続行するが、そのスレッドが生きている間は新しい移動を拒否する
    （_refuse_if_worker_alive()）。遅れて動き出した古い呼び出しと新しい移動が
    重ならないようにするため。

lab_robot.py はまだこの Mixin を使っていない（Dobot の挙動を変えないため）。
"""

import asyncio
import logging
import threading
from typing import List, Optional

logger = logging.getLogger(__name__)


class InterruptibleDriverMixin:
    """ブロッキングなドライバ呼び出しをワーカースレッドで実行し、中断に即応する"""

    # 具体クラスが __init__ で上書きする既定値
    _pose_unknown: bool = False
    _estop_generation: int = 0
    pose_settle_reads: int = 10
    pose_settle_interval: float = 0.2
    pose_settle_tolerance: float = 0.5
    stop_wait_timeout: float = 10.0
    _arm_name: str = "Arm"
    #: 最後に起動した移動ワーカーの終了イベント（未起動なら None）
    _worker_done: Optional[threading.Event] = None
    _worker_name: str = ""

    # ------------------------------------------------------------------
    # フック（具体クラスが実装する）
    # ------------------------------------------------------------------
    def _force_stop_arm(self) -> bool:  # pragma: no cover - 抽象
        raise NotImplementedError

    def _driver_pose(self) -> List[float]:  # pragma: no cover - 抽象
        raise NotImplementedError

    def _pose_indices(self, cur, prev) -> List[int]:
        return [0, 1, 2]

    def _driver_reports_idle(self) -> bool:
        return True

    def _on_pose_reread(self, pose) -> None:
        pass

    # ------------------------------------------------------------------
    # 緊急停止の世代・ワーカーの確認
    # ------------------------------------------------------------------
    def _check_estop(self, generation, where: str = "操作") -> None:
        """generation（操作の入口で取った世代）から緊急停止が入っていれば RuntimeError

        姿勢不明フラグは立てたまま（解除しない）。generation が None なら何もしない。
        """
        if generation is not None and self._estop_generation != generation:
            self._pose_unknown = True
            raise RuntimeError(f"{self._arm_name}: {where}の途中で緊急停止されました")

    def _mark_pose_known(self, generation) -> None:
        """姿勢を「既知」にする。書き込み直後に世代を確認し直す

        emergency_stop() は世代を増やしてから _pose_unknown = True を書くので、
        書き込み後に世代が変わっていなければ、その後の緊急停止の書き込みが勝つ。
        変わっていれば姿勢不明に戻して RuntimeError を送出する。
        """
        self._pose_unknown = False
        if generation is not None and self._estop_generation != generation:
            self._pose_unknown = True
            raise RuntimeError(f"{self._arm_name}: 位置確認の直後に緊急停止されました")

    def _worker_alive(self) -> bool:
        """強制停止後に放棄した移動ワーカーがまだ戻っていないか"""
        done = self._worker_done
        return done is not None and not done.is_set()

    def _refuse_if_worker_alive(self) -> None:
        """放棄したワーカーがまだ動いていれば新しい移動（・ラッチ解除）を拒否する"""
        if self._worker_alive():
            raise RuntimeError(
                f"{self._arm_name}: 中断した {self._worker_name or 'ドライバ呼び出し'} が"
                "まだ戻っていません。新しい移動は行いません（切断して再接続してください）"
            )

    # ------------------------------------------------------------------
    # 実行
    # ------------------------------------------------------------------
    async def _run_blocking(self, func, *args, moves_arm: bool = True, estop_before=None):
        """ブロッキングなドライバ呼び出しをワーカースレッドで実行する

        待機中にキャンセル（CancelledError / KeyboardInterrupt）されたら
        待機側から ``_force_stop_arm()`` を1回だけ送ってアームをその場で止め、
        ワーカーの終了を（最大 stop_wait_timeout 秒）待ってから再送出する。
        daemon スレッドを使うのは、戻らない呼び出しがプロセス終了を塞がないようにするため。

        移動（moves_arm=True）が中断・失敗した場合、姿勢は不明として扱い、
        次の移動の前にアームから読み直す（_read_pose）。
        実行中に別スレッドから緊急停止された場合（_estop_generation が変化）は、
        ドライバが正常に戻っても RuntimeError を送出して次の手順へ進ませない。

        Args:
            estop_before: 比較の基準にする緊急停止の世代（公開操作の入口で取った値）。
                None なら呼び出し時点の世代。呼び出し時点で既に変わっていれば、
                ワーカーを起動せずに RuntimeError を送出する。
        """
        name = getattr(func, "__name__", "driver")
        if estop_before is None:
            estop_before = self._estop_generation
        self._check_estop(estop_before, f"{name} の開始前")
        if moves_arm:
            self._refuse_if_worker_alive()

        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        # 待たずに終わった場合も「Future exception was never retrieved」を出さない
        fut.add_done_callback(lambda f: f.cancelled() or f.exception())
        done = threading.Event()

        def _settle(result, error):
            if not fut.done():
                if error is not None:
                    fut.set_exception(error)
                else:
                    fut.set_result(result)

        def _worker():
            try:
                result, error = func(*args), None
            except BaseException as e:  # noqa: BLE001 - 待機側へそのまま渡す
                result, error = None, e
            # 待機側が再開する前に「終了済み」にする（次の移動を誤って拒否しない）
            done.set()
            try:
                loop.call_soon_threadsafe(_settle, result, error)
            except RuntimeError:
                pass  # イベントループが既に閉じている

        if moves_arm:
            self._pose_unknown = True
            self._worker_done = done
            self._worker_name = name
        threading.Thread(
            target=_worker, name=f"{type(self).__name__}-{name}", daemon=True
        ).start()
        try:
            result = await asyncio.shield(fut)
        except (asyncio.CancelledError, KeyboardInterrupt):
            logger.warning(f"{self._arm_name}: {name} の実行中に中断されました。強制停止します...")
            self._force_stop_arm()          # 1回だけ（_wait_worker は停止を送らない）
            await self._wait_worker(fut, name)
            raise
        except Exception as e:
            if self._estop_generation != estop_before:
                # 緊急停止でドライバが例外終了した: 原因は緊急停止として報告する
                raise RuntimeError(
                    f"{self._arm_name}: {name} の実行中に緊急停止されました"
                ) from e
            raise
        if self._estop_generation != estop_before:
            # 別スレッド（GUI の Stop）から緊急停止された: ドライバが正常に戻っても
            # 移動は完了していない。姿勢は不明のまま、次の手順へ進ませない
            self._pose_unknown = True
            raise RuntimeError(f"{self._arm_name}: {name} の実行中に緊急停止されました")
        if moves_arm:
            self._mark_pose_known(estop_before)
        return result

    async def _wait_worker(self, fut, name) -> bool:
        """強制停止後、ワーカースレッドのドライバ呼び出しの終了を待つ

        期限は1回の中断につき1つ（stop_wait_timeout 秒）。待機中にさらに
        キャンセルされても停止は再送せず、同じ期限まで待ち続ける
        （呼び出し側が元の中断を再送出する）。

        Returns:
            bool: ワーカーが期限内に終了したら True
        """
        loop = asyncio.get_running_loop()
        deadline = loop.time() + float(self.stop_wait_timeout)
        while not fut.done():
            remaining = deadline - loop.time()
            if remaining <= 0:
                break
            try:
                await asyncio.wait_for(asyncio.shield(fut), remaining)
            except asyncio.TimeoutError:
                break
            except (asyncio.CancelledError, KeyboardInterrupt):
                logger.warning(
                    f"{self._arm_name}: 強制停止後の待機中に再度中断されました。"
                    f"{name} の終了を待ち続けます"
                )
                continue
            except Exception as e:
                logger.info(f"{self._arm_name}: 中断された {name} は例外で終了しました: {e}")
                return True
        if not fut.done():
            logger.error(
                f"{self._arm_name}: 強制停止後 {float(self.stop_wait_timeout):.0f} 秒待っても "
                f"{name} が戻りません。ワーカースレッドを残して中断を続行します"
                "（そのスレッドが戻るまで新しい移動は拒否します）"
            )
            return False
        if not fut.cancelled() and fut.exception() is not None:
            logger.info(f"{self._arm_name}: 中断された {name} は例外で終了しました: {fut.exception()}")
        return True

    async def _read_pose(self, estop_generation=None):
        """現在位置を取得する。前回の移動が中断・失敗していれば静止を確認して読み直す

        Args:
            estop_generation: 公開操作の入口で取った緊急停止の世代。読み取りの前後で
                比較し、変わっていたら姿勢不明のまま RuntimeError を送出する
                （None なら比較しない）

        Raises:
            RuntimeError: 位置が安定しない（アームがまだ動いている）場合、
                または読み直し中に緊急停止された場合
        """
        gen = estop_generation
        if not self._pose_unknown:
            pose = self._driver_pose()
            self._check_estop(gen, "位置の読み取り")
            return pose
        logger.warning(
            f"{self._arm_name}: 前回の移動が中断または失敗したため、アームの現在位置を読み直します..."
        )
        prev = self._driver_pose()
        self._check_estop(gen, "位置の読み直し")
        tol = self.pose_settle_tolerance
        for _ in range(max(1, int(self.pose_settle_reads))):
            await asyncio.sleep(self.pose_settle_interval)
            self._check_estop(gen, "位置の読み直し")
            cur = self._driver_pose()
            self._check_estop(gen, "位置の読み直し")
            idx = self._pose_indices(cur, prev)
            if self._driver_reports_idle() and all(abs(cur[i] - prev[i]) <= tol for i in idx):
                self._mark_pose_known(gen)
                self._on_pose_reread(cur)
                logger.info(
                    f"{self._arm_name}: 現在位置を再取得しました "
                    f"X={cur[0]:.1f}, Y={cur[1]:.1f}, Z={cur[2]:.1f}"
                )
                return cur
            prev = cur
        raise RuntimeError(
            "アームの位置が安定しません（まだ動いている可能性があります）。"
            "移動を中止しました"
        )
