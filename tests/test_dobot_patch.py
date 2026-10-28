"""pydobot_patch: 応答フレームの検証と、完了待ちのタイムアウト・停止（ハードウェア不要）。

実機の代わりに、書き込まれたフレームに応じて応答バイト列を返すフェイクの
シリアルポートを使う。応答は分割・ゴミ混入・前コマンドの遅延応答・チェックサム
不正などを再現できる。時間はフェイク時計で進めるので待ち時間は発生しない。
"""
import struct
import threading

import pytest

pytest.importorskip("pydobot")

from pydobot.dobot import Dobot
from pydobot.message import Message

from src.devices.dobot import pydobot_patch as pp
from src.devices.dobot import pydobot_controller as pc  # noqa: F401  (パッチを適用させる)

ID_GET_POSE = 10
ID_SET_PTP_CMD = 84
ID_QUEUE_INDEX = 246
POSE = (200.0, 10.0, 50.0, 5.0, 2.8, 30.0, 40.0, 2.2)


def frame(msg_id, params=b"", ctrl=0x00, bad_checksum=False):
    payload = bytes([msg_id, ctrl]) + bytes(params)
    chk = pp.checksum(payload)
    if bad_checksum:
        chk = (chk + 1) & 0xFF
    return b"\xAA\xAA" + bytes([len(payload)]) + payload + bytes([chk])


def pose_params(pose=POSE):
    return struct.pack("<8f", *pose)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, s):
        self.now += max(float(s), 0.001)


class FakeSerial:
    """書き込みごとに responder(req_id, ctrl, params) が返すチャンクを 1 回の read_all に 1 個ずつ渡す。"""

    def __init__(self, responder):
        self.responder = responder
        self.written = []
        self.pending = []        # 次の read_all で返すチャンク
        self.in_os_buffer = b""  # 送信前から OS バッファに残っているバイト
        self.resets = 0
        self.closed = False

    def write(self, data):
        data = bytes(data)
        self.written.append(data)
        assert data[:2] == b"\xAA\xAA"
        assert pp.checksum(data[3:-1]) == data[-1], "送信フレームのチェックサムが正しい"
        req_id, ctrl, params = data[3], data[4], data[5:-1]
        self.pending.extend(self.responder(req_id, ctrl, params))

    def read_all(self):
        if self.in_os_buffer:
            b, self.in_os_buffer = self.in_os_buffer, b""
            return b
        return self.pending.pop(0) if self.pending else b""

    def reset_input_buffer(self):
        self.resets += 1
        self.in_os_buffer = b""

    def close(self):
        self.closed = True


def make_dobot(responder):
    dev = object.__new__(Dobot)
    dev.ser = FakeSerial(responder)
    dev.lock = threading.Lock()
    dev.verbose = False
    return dev


@pytest.fixture(autouse=True)
def clock(monkeypatch):
    c = FakeClock()
    monkeypatch.setattr(pp, "time", c)
    return c


def pose_request():
    m = Message()
    m.id = ID_GET_POSE
    return m


# ---------------------------------------------------------------------------
# 応答の整合性
# ---------------------------------------------------------------------------
def test_split_frame_is_reassembled():
    full = frame(ID_GET_POSE, pose_params())
    dev = make_dobot(lambda i, c, p: [full[:3], full[3:10], full[10:]])
    assert dev.pose() == pytest.approx(POSE)


def test_garbage_around_frames_is_skipped():
    full = frame(ID_GET_POSE, pose_params())
    dev = make_dobot(lambda i, c, p: [b"\x00\x13\xAA", b"\x55" + full[:7], full[7:] + b"\xFF"])
    assert dev.pose() == pytest.approx(POSE)


def test_stale_reply_from_previous_id_is_discarded():
    """前コマンドの遅延応答（別 ID）を次の応答として消費しない。"""
    stale = frame(ID_QUEUE_INDEX, struct.pack("<Q", 7))
    good = frame(ID_GET_POSE, pose_params())
    dev = make_dobot(lambda i, c, p: [stale, good])
    assert dev.pose() == pytest.approx(POSE)


def test_stale_same_id_pose_before_send_is_flushed():
    """送信前にバッファに残っていた古い pose 応答は捨てられ、新しい応答が使われる。"""
    old_pose = (1.0, 2.0, 3.0, 4.0, 0, 0, 0, 0)
    dev = make_dobot(lambda i, c, p: [frame(ID_GET_POSE, pose_params())])
    dev.ser.in_os_buffer = frame(ID_GET_POSE, pose_params(old_pose))
    dev._rx_buf = bytearray(frame(ID_GET_POSE, pose_params(old_pose))[:5])
    assert dev.pose() == pytest.approx(POSE)
    assert dev.ser.resets == 1


def test_bad_checksum_frame_is_discarded():
    bad = frame(ID_GET_POSE, pose_params((9, 9, 9, 9, 9, 9, 9, 9)), bad_checksum=True)
    good = frame(ID_GET_POSE, pose_params())
    dev = make_dobot(lambda i, c, p: [bad, good])
    assert dev.pose() == pytest.approx(POSE)


def test_only_bad_checksum_raises_instead_of_returning_garbage():
    bad = frame(ID_GET_POSE, pose_params(), bad_checksum=True)
    dev = make_dobot(lambda i, c, p: [bad])
    with pytest.raises(pp.DobotReplyError):
        dev.pose()


def test_no_reply_raises_after_read_timeout(clock):
    dev = make_dobot(lambda i, c, p: [])
    with pytest.raises(pp.DobotReplyError):
        dev.pose()
    assert clock.now >= pp.DEFAULT_READ_TIMEOUT_S


def test_mismatched_reply_only_raises():
    dev = make_dobot(lambda i, c, p: [frame(ID_QUEUE_INDEX, struct.pack("<Q", 1))])
    with pytest.raises(pp.DobotReplyError):
        dev.pose()


def test_corrupted_length_does_not_block_following_valid_frame():
    good = frame(ID_GET_POSE, pose_params())
    dev = make_dobot(lambda i, c, p: [b"\xAA\xAA\xF0\x0A", good])
    assert dev.pose() == pytest.approx(POSE)


def test_extract_frames_keeps_incomplete_tail():
    good = frame(ID_GET_POSE, pose_params())
    buf = bytearray(good + good[:6])
    assert pp.extract_frames(buf) == [good]
    assert bytes(buf) == good[:6]


def test_outgoing_checksum_is_twos_complement():
    """上流の % 255 は和が 256 の倍数のとき 1 を送っていた。"""
    m = Message()
    m.id = 0xF0
    m.ctrl = 0x10          # 0xF0 + 0x10 = 256
    m.params = bytearray()
    assert pp.encode_frame(m)[-1] == 0x00
    m2 = Message()
    m2.id = ID_GET_POSE
    m2.ctrl = 0
    m2.params = bytearray()
    assert pp.encode_frame(m2) == bytes(Message.bytes(m2))


def test_queue_index_widths():
    assert pp.queue_index(struct.pack("<Q", 2 ** 40)) == 2 ** 40
    assert pp.queue_index(struct.pack("<I", 17)) == 17
    with pytest.raises(pp.DobotReplyError):
        pp.queue_index(b"\x01")


# ---------------------------------------------------------------------------
# 完了待ち (wait=True)
# ---------------------------------------------------------------------------
class QueueFirmware:
    """SET_PTP_CMD に index を払い出し、GET_QUEUED_CMD_CURRENT_INDEX に現在値を返す。"""

    def __init__(self, advance, expected=100):
        self.expected = expected
        self.advance = advance      # polls -> current index
        self.polls = 0
        self.on_poll = None

    def __call__(self, req_id, ctrl, params):
        if req_id == ID_SET_PTP_CMD:
            return [frame(ID_SET_PTP_CMD, struct.pack("<Q", self.expected), ctrl)]
        if req_id == ID_QUEUE_INDEX:
            self.polls += 1
            if self.on_poll:
                self.on_poll(self.polls)
            return [frame(ID_QUEUE_INDEX, struct.pack("<Q", self.advance(self.polls)))]
        if req_id == ID_GET_POSE:
            return [frame(ID_GET_POSE, pose_params())]
        return [frame(req_id, b"", ctrl)]


def test_wait_done_when_index_passes_expected():
    """index が期待値を飛び越えても（== でなく >=）完了とみなす。"""
    fw = QueueFirmware(lambda n: 99 if n < 3 else 105)
    dev = make_dobot(fw)
    dev.move_to(200, 0, 50, 0, wait=True)
    assert fw.polls == 3


def test_wait_times_out_instead_of_hanging(clock):
    fw = QueueFirmware(lambda n: 0)     # force_stop + clear 後に index が戻った状況
    dev = make_dobot(fw)
    dev.move_timeout_s = 5.0
    with pytest.raises(pp.DobotWaitTimeout):
        dev.move_to(200, 0, 50, 0, wait=True)
    assert 5.0 <= clock.now < 10.0


def test_wait_aborts_immediately_on_stop_request():
    fw = QueueFirmware(lambda n: 0)
    dev = make_dobot(fw)
    fw.on_poll = lambda n: pp.request_stop(dev) if n == 2 else None
    with pytest.raises(pp.DobotMoveAborted):
        dev.move_to(200, 0, 50, 0, wait=True)
    assert fw.polls == 2


def test_new_move_clears_previous_stop_request():
    fw = QueueFirmware(lambda n: 100)
    dev = make_dobot(fw)
    pp.request_stop(dev)
    dev.move_to(200, 0, 50, 0, wait=True)     # 例外にならない
    assert not pp.stop_requested(dev)


def test_non_wait_commands_keep_stop_request():
    """force_stop 自身の即時コマンドやポーリングでは停止フラグを下ろさない。"""
    dev = make_dobot(QueueFirmware(lambda n: 0))
    pp.request_stop(dev)
    dev._get_queued_cmd_current_index()
    dev._set_queued_cmd_clear()
    assert pp.stop_requested(dev)


def test_transient_poll_failure_does_not_abort_wait():
    fw = QueueFirmware(lambda n: 100)
    base = fw.__call__

    def flaky(req_id, ctrl, params):
        if req_id == ID_QUEUE_INDEX and fw.polls == 0:
            fw.polls += 1
            return []            # 1 回目のポーリングだけ無応答
        return base(req_id, ctrl, params)

    dev = make_dobot(flaky)
    dev.move_to(200, 0, 50, 0, wait=True)
    assert fw.polls == 2


def test_dobot_init_failure_closes_serial(monkeypatch):
    """初期化中に応答が無ければ COM ポートを閉じてから例外を出す。"""
    import pydobot.dobot as upstream

    created = []

    def fake_serial(port, **kw):
        s = FakeSerial(lambda i, c, p: [])
        s.isOpen = lambda: True
        s.name = port
        created.append(s)
        return s

    monkeypatch.setattr(upstream.serial, "Serial", fake_serial)
    with pytest.raises(pp.DobotReplyError):
        Dobot(port="COMX")
    assert created and created[0].closed


def test_patch_is_idempotent():
    init = Dobot.__init__
    import pydobot.dobot as upstream
    module = upstream.serial
    assert pp.apply_pydobot_patch()
    assert Dobot.__init__ is init
    assert upstream.serial is module


def test_upstream_opens_port_without_rts_dtr(monkeypatch):
    """pydobot の open は RTS / DTR を下げた状態で行われる（接続時のリセット対策）。"""
    import pydobot.dobot as upstream
    import serial

    events = []

    class RecordingSerial:
        def __setattr__(self, name, value):
            events.append((name, value))
            object.__setattr__(self, name, value)

        def open(self):
            events.append(("open", None))

    monkeypatch.setattr(serial, "Serial", RecordingSerial)
    assert upstream.serial.Serial is pp.open_serial_quiet
    assert upstream.serial.PARITY_NONE == serial.PARITY_NONE   # 定数は本物の serial から
    upstream.serial.Serial("COMX", baudrate=115200, bytesize=serial.EIGHTBITS)
    names = [name for name, _ in events]
    assert names[-1] == "open"
    assert ("dtr", False) in events and ("rts", False) in events
    assert names.index("dtr") < names.index("open")
    assert names.index("rts") < names.index("open")
    assert ("port", "COMX") in events and ("baudrate", 115200) in events


# ---------------------------------------------------------------------------
# PyDobotController 経由（相対移動の目標が正しい pose から計算される）
# ---------------------------------------------------------------------------
def test_move_z_uses_matching_pose_reply():
    stale = frame(ID_QUEUE_INDEX, struct.pack("<Q", 3))
    moves = []

    def fw(req_id, ctrl, params):
        if req_id == ID_GET_POSE:
            return [stale, frame(ID_GET_POSE, pose_params())]
        if req_id == ID_SET_PTP_CMD:
            moves.append(struct.unpack("<B4f", bytes(params)))
            return [frame(ID_SET_PTP_CMD, struct.pack("<Q", 5), ctrl)]
        if req_id == ID_QUEUE_INDEX:
            return [frame(ID_QUEUE_INDEX, struct.pack("<Q", 5))]
        return [frame(req_id, b"", ctrl)]

    ctrl = object.__new__(pc.PyDobotController)
    ctrl.port_name = "TEST"
    ctrl.device = make_dobot(fw)
    ctrl._connected = True
    ctrl.verbose = False
    ctrl.move_Z(-10)
    assert moves[0][1:] == pytest.approx((POSE[0], POSE[1], POSE[2] - 10, POSE[3]))


# ---------------------------------------------------------------------------
# アラーム（2026-10-28: 0x22 で止まったのに「移動完了」になった）
# ---------------------------------------------------------------------------
ID_ALARMS = 20


def alarm_params(*codes):
    raw = bytearray(16)
    for c in codes:
        raw[c // 8] |= 1 << (c % 8)
    return bytes(raw)


def move_responder(alarms=(), seen=None):
    def fw(req_id, ctrl, params):
        if seen is not None:
            seen.append(req_id)
        if req_id == ID_SET_PTP_CMD:
            return [frame(ID_SET_PTP_CMD, struct.pack("<Q", 5), ctrl)]
        if req_id == ID_QUEUE_INDEX:
            return [frame(ID_QUEUE_INDEX, struct.pack("<Q", 5))]   # index reached
        if req_id == ID_ALARMS:
            return [frame(ID_ALARMS, alarm_params(*alarms))]
        if req_id == ID_GET_POSE:
            return [frame(ID_GET_POSE, pose_params())]
        return [frame(req_id, b"", ctrl)]
    return fw


def test_alarm_bits():
    assert pp.alarm_bits(alarm_params(0x22)) == [0x22]
    assert pp.alarm_bits(alarm_params(0x04, 0x55)) == [0x04, 0x55]
    assert pp.alarm_bits(bytes(16)) == []


def test_move_with_alarm_raises_although_the_queue_finished():
    seen = []
    dev = make_dobot(move_responder(alarms=(0x22,), seen=seen))
    with pytest.raises(pp.DobotAlarmError, match="0x22 motion: inverse kinematics limit") as e:
        dev.move_to(281.0, 13.0, 114.8, 0.0, wait=True)
    assert e.value.alarms == [0x22]
    assert seen[-1] == ID_ALARMS            # checked after the index was reached


def test_move_without_alarm_completes_and_checks_once():
    seen = []
    dev = make_dobot(move_responder(seen=seen))
    dev.move_to(281.0, 13.0, 114.8, 0.0, wait=True)
    assert seen.count(ID_ALARMS) == 1


def test_immediate_commands_do_not_read_alarms():
    seen = []
    dev = make_dobot(move_responder(alarms=(0x22,), seen=seen))
    assert dev.pose() == pytest.approx(POSE)
    assert ID_ALARMS not in seen


def test_unanswered_alarm_query_fails_safe():
    def fw(req_id, ctrl, params):
        if req_id == ID_ALARMS:
            return []
        return move_responder()(req_id, ctrl, params)

    dev = make_dobot(fw)
    with pytest.raises(pp.DobotReplyError, match="GetAlarmsState"):
        dev.move_to(281.0, 13.0, 114.8, 0.0, wait=True)


def test_alarm_reaches_the_controller_caller():
    """PyDobotController.move_XYZ_abs で例外として上がる（LabRobot はそれを再送出する）。"""
    ctrl = object.__new__(pc.PyDobotController)
    ctrl.port_name = "TEST"
    ctrl.device = make_dobot(move_responder(alarms=(0x22,)))
    ctrl._connected = True
    ctrl.verbose = False
    with pytest.raises(pp.DobotAlarmError):
        ctrl.move_XYZ_abs(281.0, 13.0, 114.8)
