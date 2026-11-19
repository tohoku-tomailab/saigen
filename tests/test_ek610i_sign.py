"""Balance driver: A&D EK-610i (`Ek610i.Scale`) の応答で符号が失われないこと。"""
import pytest

from tests.test_wp2_picus2_connection import _install_stubs  # noqa: F401  (serial スタブ)

_install_stubs()

from src.devices.scale.Ek610i import Scale  # noqa: E402


class _Ser:
    """`Scale._ser` の代わり: 固定の応答行を返すだけ。"""

    def __init__(self, line: bytes):
        self.line = line

    def reset_input_buffer(self):
        pass

    def write(self, data):
        pass

    def flush(self):
        pass

    def readline(self):
        return self.line


def _scale(line: bytes) -> Scale:
    scale = Scale.__new__(Scale)
    scale._ser = _Ser(line)
    return scale


@pytest.mark.parametrize(
    "raw,expected",
    [
        (b"ST,-0000.03 g\r\n", -0.03),      # 負値（本件の再現ケース）
        (b"ST,+0012.34 g\r\n", 12.34),
        (b"US,-0001.25 g\r\n", -1.25),      # 不安定マーカー付き
        (b"ST,+0000.00 g\r\n", 0.0),
    ],
)
def test_query_weight_preserves_sign(raw, expected):
    assert _scale(raw).query_weight() == pytest.approx(expected)
