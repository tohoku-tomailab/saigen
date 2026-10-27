"""A&D EK-610i アダプタ: 応答の読み取りと SharedDevices での機種切り替え。"""
import asyncio
import sys
import types

import pytest

from tests.test_wp2_picus2_connection import _install_stubs  # noqa: F401  (serial スタブ)

_install_stubs()

from src.devices.scale import ek610i_balance  # noqa: E402
from src.devices.scale.ek610i_balance import EK610iBalance, parse_reply  # noqa: E402


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("ST,+00000.02  g", 0.02),        # 実機 (EK-610i) の応答
        ("ST,+00123.45  g", 123.45),
        ("ST,-00000.03  g", -0.03),
        ("US,+00010.50  g", 10.5),        # 不安定でも値は使う（BCE8221 と同じ扱い）
        ("ST,-  0.03 g", -0.03),          # 符号と数字の間に空白
        ("ST,+00000.02  g\r\n", 0.02),
    ],
)
def test_parse_reply_reads_weight(raw, expected):
    assert parse_reply(raw) == pytest.approx(expected)


@pytest.mark.parametrize(
    "raw",
    [
        "",
        "OL,+9999999E+19",                # 過負荷
        "QT,+0000123  PC",                # 個数モード
        "ST,+00000.02 ct",                # g 以外の単位
        "ST,+00000.02  oz",
        "EC,E11",                         # エラーコード
        "garbage",
    ],
)
def test_parse_reply_rejects_non_weight(raw):
    assert parse_reply(raw) is None


class _Ser:
    def __init__(self, reply=b"ST,+00000.02  g\r\n"):
        self.reply = reply
        self.written = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)

    def readline(self):
        return self.reply

    def close(self):
        self.closed = True


def _balance(ser):
    bal = object.__new__(EK610iBalance)
    bal.ser = ser
    return bal


def test_get_weight_sends_q_and_parses():
    ser = _Ser(b"ST,-00001.25  g\r\n")
    assert _balance(ser).get_weight() == pytest.approx(-1.25)
    assert ser.written == [b"Q\r\n"]


def test_get_weight_returns_none_on_silence():
    assert _balance(_Ser(b"")).get_weight() is None


def test_tare_sends_r_and_close_closes():
    ser = _Ser()
    bal = _balance(ser)
    bal.tare()
    bal.close()
    assert ser.written == [b"R\r\n"]
    assert ser.closed


def test_opens_port_with_ad_factory_settings(monkeypatch):
    seen = {}

    def _serial(**kw):
        seen.update(kw)
        return _Ser()

    monkeypatch.setattr(ek610i_balance.serial, "Serial", _serial, raising=False)
    EK610iBalance(port="COM9")
    assert seen["port"] == "COM9"
    assert seen["baudrate"] == 2400
    assert seen["bytesize"] == ek610i_balance.serial.SEVENBITS
    assert seen["parity"] == ek610i_balance.serial.PARITY_EVEN


def _shared(**configs):
    from src.devices.safety.shared_devices import SharedDevices
    return SharedDevices(use_scale=True, scale_port="COM9", **configs)


def _fake_module(monkeypatch, name, attr):
    created = []

    class _Fake:
        def __init__(self, port):
            created.append(port)

        def get_weight(self):
            return 1.0

    mod = types.ModuleType(name)
    setattr(mod, attr, _Fake)
    monkeypatch.setitem(sys.modules, name, mod)
    return created


def test_shared_devices_defaults_to_bce8221(monkeypatch):
    bce = _fake_module(monkeypatch, "src.devices.scale.BCE8221", "SerialBalance")
    ek = _fake_module(monkeypatch, "src.devices.scale.ek610i_balance", "EK610iBalance")
    assert asyncio.run(_shared()._initialize_scale())
    assert bce == ["COM9"] and ek == []


def test_shared_devices_selects_ek610i(monkeypatch):
    bce = _fake_module(monkeypatch, "src.devices.scale.BCE8221", "SerialBalance")
    ek = _fake_module(monkeypatch, "src.devices.scale.ek610i_balance", "EK610iBalance")
    assert asyncio.run(_shared(scale_type="EK610i")._initialize_scale())
    assert ek == ["COM9"] and bce == []


def test_shared_devices_rejects_unknown_scale_type(monkeypatch):
    bce = _fake_module(monkeypatch, "src.devices.scale.BCE8221", "SerialBalance")
    ek = _fake_module(monkeypatch, "src.devices.scale.ek610i_balance", "EK610iBalance")
    assert not asyncio.run(_shared(scale_type="mettler")._initialize_scale())
    assert bce == [] and ek == []


def test_session_factory_passes_scale_type():
    from src.flow.experiment_session import default_shared_factory

    shared = default_shared_factory(
        use_scale=True, use_camera=False,
        config={"scale_port": "COM9", "scale_type": "ek610i", "camera_index": 0},
    )
    assert shared.device_configs["scale_type"] == "ek610i"
