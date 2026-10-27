"""
pydobot (1.3.2) の通信部分を差し替えるモンキーパッチ

差し替えるメソッドと理由:

``Dobot._send_message`` / ``Dobot._read_message`` （応答の取り違え対策）
    上流の _read_message は 100ms 後にバッファにあるバイトを丸ごと 1 フレームとして
    解釈し、ヘッダ・長さ・チェックサムも応答 ID も確認しない。遅れて届いた前の
    コマンドの応答が次のコマンドの応答として消費されると、move_Z / move_XY が
    pose() から計算する相対移動の目標が（範囲内だが）誤った値になる。
    このパッチは

    * 送信前に受信バッファ（OS 側と自前の _rx_buf）を捨て、
    * 受信バイトを _rx_buf に溜めて ``AA AA len id ctrl params chk`` で切り出し、
    * 長さ・チェックサム不正のフレームを捨て、
    * 応答 ID が要求 ID と一致するフレームだけを返す（不一致は捨てて待ち直す）。

    ``timeout`` 秒以内に一致する応答が来なければ None を返し、_send_command が
    DobotReplyError を送出する。
    送信フレームのチェックサムも正しい 2 の補数で作り直す（上流の Message.refresh は
    ``% 255`` のため、ペイロード和が 256 の倍数 / +1 のときに誤った値を送る）。

``Dobot._send_command`` （緊急停止後のハング対策）
    上流は wait=True のとき現在のキュー index が期待値に「等しくなる」まで
    タイムアウト無しでポーリングする。force_stop（QueuedCmdForceStopExec + キュー
    破棄）の後はその index に到達しないことがあり、アームは止まっているのに実行
    スレッドが永久に待ち続けた。このパッチは

    * ``current >= expected`` で完了とみなし、
    * ``move_timeout_s``（既定 60 s、Dobot インスタンス属性で変更可）で打ち切って
      DobotWaitTimeout を送出し、
    * request_stop() で立てた停止フラグを見たら即座に DobotMoveAborted を送出する。

    停止フラグは次に wait=True のコマンド（＝呼び出し側が明示的に次の動作を指示した
    とき）を送る直前に clear_stop() で下ろす。force_stop 自身が送る即時コマンドや
    キュー index のポーリングでは下ろさない（下ろすと待機側が停止に気付けない）。

    同じ index 値の応答どうし（連続する GET_QUEUED_CMD_CURRENT_INDEX など）は
    プロトコルに通番が無いため区別できない。送信前のフラッシュで大半は防げる。

``Dobot.__init__``
    初期化コマンドが失敗したときに、開いたシリアルポートを閉じてから例外を再送出
    する（Windows では閉じないと COM ポートが掴まれたままになる）。

``pydobot.dobot.serial.Serial`` （接続時のリセット・無応答対策）
    pyserial の既定の open は RTS / DTR を立てる。CP2102N の個体ではこれでコントローラ
    がリセットされ、CH9102 (1A86:55D4) の個体でも、上流の open の直後から電源を入れ
    直すまで一切応答しなくなった（2026-10-27, 最初のコマンド id=240 が無応答）。
    read_pose.open_quiet と同じく、RTS / DTR を下げたまま開く。

著作権表示:
    差し替え対象のメソッドは pydobot (https://github.com/luismesas/pydobot)
    Copyright 2017 Luis Mesas, MIT License に由来する。全文は
    リポジトリ直下の THIRD_PARTY_NOTICES.md を参照。
"""

import logging
import struct
import threading
import time

logger = logging.getLogger(__name__)

HEADER = b"\xAA\xAA"
DEFAULT_READ_TIMEOUT_S = 2.0
DEFAULT_MOVE_TIMEOUT_S = 60.0
QUEUE_POLL_INTERVAL_S = 0.1
ID_GET_QUEUED_CMD_CURRENT_INDEX = 246


class DobotReplyError(ConnectionError):
    """一致する応答がタイムアウト内に得られなかった。"""


class DobotWaitTimeout(TimeoutError):
    """キュー付きコマンドの完了待ちが move_timeout_s を超えた。"""


class DobotMoveAborted(RuntimeError):
    """force_stop() による停止要求で完了待ちを打ち切った。"""


# ---------------------------------------------------------------------------
# 停止フラグ（Dobot インスタンスごと）
# ---------------------------------------------------------------------------
def _stop_event(dev) -> threading.Event:
    ev = getattr(dev, "_stop_event", None)
    if ev is None:
        ev = dev.__dict__.setdefault("_stop_event", threading.Event())
    return ev


def request_stop(dev) -> None:
    """完了待ち中のスレッドに停止を知らせる（force_stop から呼ぶ）。"""
    _stop_event(dev).set()


def clear_stop(dev) -> None:
    _stop_event(dev).clear()


def stop_requested(dev) -> bool:
    return _stop_event(dev).is_set()


# ---------------------------------------------------------------------------
# フレーム処理（純粋関数）
# ---------------------------------------------------------------------------
def _as_int(v) -> int:
    return v.value if hasattr(v, "value") else int(v)


def checksum(payload: bytes) -> int:
    """Dobot プロトコルのチェックサム: ペイロード (id, ctrl, params) 和の 2 の補数。"""
    return (-sum(payload)) & 0xFF


def encode_frame(msg) -> bytes:
    payload = bytes([_as_int(msg.id), _as_int(msg.ctrl)]) + bytes(msg.params)
    return HEADER + bytes([len(payload)]) + payload + bytes([checksum(payload)])


def _valid_frame_at(buf: bytearray, i: int) -> bool:
    if len(buf) < i + 4 or buf[i + 2] < 2:
        return False
    end = i + 3 + buf[i + 2]
    return len(buf) > end and checksum(bytes(buf[i + 3:end])) == buf[end]


def extract_frames(buf: bytearray) -> list:
    """buf から完全なフレームを切り出して返し、消費した分を buf から除く。

    ヘッダ前のゴミ、長さ不正・チェックサム不正のフレームは捨てる。末尾の
    未完成フレームは buf に残す。戻り値は生フレーム（bytes）のリスト。
    """
    frames = []
    while True:
        start = buf.find(HEADER)
        if start < 0:
            # 最後の 1 バイトが 0xAA ならヘッダの前半かもしれないので残す
            keep = 1 if buf[-1:] == b"\xAA" else 0
            del buf[:len(buf) - keep]
            return frames
        if start:
            logger.debug(f"pydobot: discarding {start} byte(s) before header")
            del buf[:start]
        if len(buf) < 3:
            return frames
        length = buf[2]
        if length < 2:
            logger.warning("pydobot: discarding frame with invalid length %d", length)
            del buf[:2]
            continue
        total = 3 + length + 1
        if len(buf) < total:
            # 長さバイトが壊れていると、来ない残りを待ち続けてしまう。後ろに完全で
            # 妥当なフレームがあれば、この候補はゴミとみなして捨てる。
            nxt = buf.find(HEADER, 2)
            if nxt > 0 and _valid_frame_at(buf, nxt):
                logger.warning("pydobot: discarding truncated frame before a valid one")
                del buf[:nxt]
                continue
            return frames
        payload = bytes(buf[3:3 + length])
        chk = buf[3 + length]
        if checksum(payload) != chk:
            logger.warning("pydobot: discarding frame with bad checksum (id=%d)", payload[0])
            del buf[:2]   # ヘッダだけ捨てて再同期する
            continue
        frames.append(bytes(buf[:total]))
        del buf[:total]


def _rx_buf(dev) -> bytearray:
    buf = getattr(dev, "_rx_buf", None)
    if buf is None:
        buf = dev.__dict__.setdefault("_rx_buf", bytearray())
    return buf


def _flush_input(dev) -> None:
    _rx_buf(dev).clear()
    try:
        dev.ser.reset_input_buffer()
    except Exception:
        stale = dev.ser.read_all()
        if stale:
            logger.debug(f"pydobot: flushed {len(stale)} stale byte(s)")


def queue_index(params) -> int:
    """キュー index（uint64 LE、古いファームでは uint32）を読む。"""
    raw = bytes(params)
    if len(raw) >= 8:
        return struct.unpack_from("<Q", raw, 0)[0]
    if len(raw) >= 4:
        return struct.unpack_from("<I", raw, 0)[0]
    raise DobotReplyError(f"Dobot: queue index reply too short ({len(raw)} bytes)")


# ---------------------------------------------------------------------------
# 差し替えメソッド
# ---------------------------------------------------------------------------
def _patched_send_message(self, msg):
    """送信前に受信側を空にし、正しいチェックサムで送る（上流と同じ 100ms 間隔）。"""
    time.sleep(0.1)
    _flush_input(self)
    frame = encode_frame(msg)
    if self.verbose:
        logger.info(f"pydobot: >> {frame.hex(' ')}")
    self.ser.write(frame)


def _patched_read_message(self, timeout=DEFAULT_READ_TIMEOUT_S, poll_interval=0.01,
                          expected_id=None):
    """expected_id と一致する妥当なフレームを最大 timeout 秒待つ。無ければ None。"""
    from pydobot.message import Message

    buf = _rx_buf(self)
    deadline = time.monotonic() + timeout
    while True:
        chunk = self.ser.read_all()
        if chunk:
            buf.extend(chunk)
            for frame in extract_frames(buf):
                msg = Message(frame)
                if expected_id is not None and msg.id != expected_id:
                    logger.warning(
                        f"pydobot: discarding reply id={msg.id} (waiting for id={expected_id})"
                    )
                    continue
                if self.verbose:
                    logger.info(f"pydobot: << {frame.hex(' ')}")
                return msg
        if time.monotonic() >= deadline:
            return None
        time.sleep(poll_interval)


def _patched_get_queued_cmd_current_index(self):
    from pydobot.message import Message
    msg = Message()
    msg.id = ID_GET_QUEUED_CMD_CURRENT_INDEX
    return queue_index(self._send_command(msg).params)


def _patched_send_command(self, msg, wait=False):
    if wait:
        clear_stop(self)   # 呼び出し側が新しい動作を指示した → 以前の停止要求を下ろす
    msg_id = _as_int(msg.id)
    with self.lock:
        self._send_message(msg)
        response = self._read_message(expected_id=msg_id)
    if response is None:
        raise DobotReplyError(f"Dobot: no valid reply to command id={msg_id}")
    if not wait:
        return response

    expected = queue_index(response.params)
    timeout = float(getattr(self, "move_timeout_s", None) or DEFAULT_MOVE_TIMEOUT_S)
    deadline = time.monotonic() + timeout
    current = None
    while True:
        if stop_requested(self):
            raise DobotMoveAborted(
                f"Dobot: wait for queue index {expected} aborted by force_stop"
            )
        try:
            current = self._get_queued_cmd_current_index()
        except DobotReplyError as e:   # 一時的な通信失敗は期限まで待ち直す
            logger.warning(f"pydobot: queue index poll failed: {e}")
        if current is not None and current >= expected:
            return response
        if time.monotonic() >= deadline:
            raise DobotWaitTimeout(
                f"Dobot: command (queue index {expected}) not finished within "
                f"{timeout:.0f}s (current index {current})"
            )
        time.sleep(QUEUE_POLL_INTERVAL_S)


def open_serial_quiet(port, **settings):
    """``serial.Serial(port, **settings)`` と同じポートを、RTS / DTR を立てずに開く。"""
    import serial
    ser = serial.Serial()
    ser.port = port
    for name, value in settings.items():
        setattr(ser, name, value)
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def _quiet_serial_module():
    """Serial だけを open_serial_quiet に差し替え、ほかは本物の serial を見せるモジュール。"""
    import types

    import serial

    module = types.ModuleType("serial_quiet_open")
    module.__getattr__ = lambda name: getattr(serial, name)
    module.Serial = open_serial_quiet
    module._saigen_patched = True
    return module


def _wrap_init(orig_init):
    def __init__(self, *args, **kwargs):
        try:
            orig_init(self, *args, **kwargs)
        except BaseException:
            ser = getattr(self, "ser", None)
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            raise
    __init__._saigen_patched = True
    return __init__


def apply_pydobot_patch() -> bool:
    """pydobot.Dobot をパッチする。import の直後に1回呼ぶ（2回目以降は何もしない）。

    pydobot 未インストール環境（テスト・モック実行）では import 失敗で
    モジュール全体が読み込めなくなるのを避けるため、False を返すだけにする。
    """
    try:
        import pydobot.dobot as upstream
        from pydobot.dobot import Dobot
    except ImportError:
        logger.warning("pydobot not installed: patch skipped")
        return False
    if not getattr(upstream.serial, "_saigen_patched", False):
        upstream.serial = _quiet_serial_module()
    Dobot._send_message = _patched_send_message
    Dobot._read_message = _patched_read_message
    Dobot._send_command = _patched_send_command
    Dobot._get_queued_cmd_current_index = _patched_get_queued_cmd_current_index
    if not getattr(Dobot.__init__, "_saigen_patched", False):
        Dobot.__init__ = _wrap_init(Dobot.__init__)
    Dobot.move_timeout_s = DEFAULT_MOVE_TIMEOUT_S
    return True
