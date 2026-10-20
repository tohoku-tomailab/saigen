import logging
import serial
import time
import re

logger = logging.getLogger(__name__)


class Scale:
    """
    A&D EK-610i 系電子天びんを RS-232C 経由で操作するクラス。
    コンテキストマネージャ対応で、自動的にポートを開閉します。
    """

    DEFAULT_PORT     = "COM3"
    DEFAULT_BAUD     = 2400
    DEFAULT_BYTESIZE = serial.SEVENBITS
    DEFAULT_PARITY   = serial.PARITY_EVEN
    DEFAULT_STOPBITS = serial.STOPBITS_ONE
    READ_TIMEOUT     = 2     # 秒
    WRITE_TIMEOUT    = 1     # 秒

    def __init__(self,
                 port: str = DEFAULT_PORT,
                 baudrate: int = DEFAULT_BAUD,
                 bytesize: int = DEFAULT_BYTESIZE,
                 parity: str = DEFAULT_PARITY,
                 stopbits: int = DEFAULT_STOPBITS):
        self.port     = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity   = parity
        self.stopbits = stopbits
        self._ser     = None

    def __enter__(self):
        self._ser = serial.Serial(
            port        = self.port,
            baudrate    = self.baudrate,
            bytesize    = self.bytesize,
            parity      = self.parity,
            stopbits    = self.stopbits,
            timeout     = self.READ_TIMEOUT,
            write_timeout = self.WRITE_TIMEOUT,
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _send_command(self, cmd: bytes):
        """CR+LF 終端付きでコマンドを送信し、バッファをクリア→フラッシュ"""
        self._ser.reset_input_buffer()
        self._ser.write(cmd + b"\r\n")
        self._ser.flush()

    def query_weight(self) -> float:
        """
        計量値を要求し、float で返却。
        応答がなければ TimeoutError を投げる。
        """
        self._send_command(b"Q")
        raw = self._ser.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            raise TimeoutError("天びんから応答がありません")
        # 例: "ST,+000123.45 g" / "ST,-0000.03 g"
        header, val_unit = raw.split(",", 1)
        # 符号付きの数値部分を抽出（"[^\d.]" を削る方式は符号を落としていた）
        match = re.search(r"[+-]?\d+(?:\.\d+)?", val_unit)
        if match is None:
            raise ValueError(f"計量値を解釈できません: {raw!r}")
        return float(match.group())

    def tare(self, delay: float = 0.5):
        """
        風袋引き（ゼロ点）コマンド。
        delay 秒だけ待ってから戻る。
        """
        self._send_command(b"R")
        time.sleep(delay)


if __name__ == "__main__":
    with Scale(port="COM3") as scale:
        # 必要に応じて風袋引き
        # scale.tare()

        # 重量取得
        weight = scale.query_weight()
        print(f"計量値: {weight:.2f} g")
