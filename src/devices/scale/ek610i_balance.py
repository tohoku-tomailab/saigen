"""A&D EK-i 系（EK-610i など）電子天びんを SerialBalance と同じ形で使うアダプタ。

SharedDevices は天びんに ``get_weight()`` / ``tare()`` / ``close()`` だけを求める。
BCE8221 の ``SerialBalance`` と同じ形にそろえ、``config.yaml`` の
``shared_devices.scale_type: ek610i`` で切り替えられるようにする。

通信設定は A&D の工場出荷値（2400 bps, 7 bit, 偶数パリティ, 1 stop bit）。
応答の形式は ``HD,±nnnnn.nn  g``（HD = ST 安定 / US 不安定 / QT 個数 / OL 過負荷）。
"""
import logging
import re

import serial

logger = logging.getLogger(__name__)

_REPLY = re.compile(r"^(ST|US|QT|OL),\s*([+-])?\s*(\d+(?:\.\d+)?)\s*(\S*)\s*$")


def parse_reply(line: str):
    """A&D の応答 1 行を重量 (g) に変換する。読めない・過負荷・g 以外の単位は None。"""
    match = _REPLY.match(line.strip())
    if not match:
        return None
    header, sign, number, unit = match.groups()
    if header != "ST" and header != "US":
        return None  # OL（過負荷）と QT（個数モード）は重量ではない
    if unit != "g":
        return None  # 単位が g 以外の設定では値を使わない
    return (-1.0 if sign == "-" else 1.0) * float(number)


class EK610iBalance:
    def __init__(self, port='COM4', baudrate=2400, timeout=2):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.SEVENBITS,
            timeout=timeout,
        )

    def get_weight(self):
        """天びんから重量データを取得

        Returns:
            float | None: 重量(g)。数値が読めなかった場合は None
        """
        self.ser.reset_input_buffer()
        self.ser.write(b'Q\r\n')  # 即時計量値の要求
        raw = self.ser.readline().decode('ascii', errors='replace').strip()
        number = parse_reply(raw)
        if number is None:
            logger.warning(f"天びんの応答から重量を読み取れませんでした: {raw!r}")
        return number

    def tare(self):
        """風袋引き（ゼロ点リセット）を実行"""
        self.ser.write(b'R\r\n')  # RE-ZERO コマンド

    def close(self):
        """シリアル通信を終了"""
        self.ser.close()
