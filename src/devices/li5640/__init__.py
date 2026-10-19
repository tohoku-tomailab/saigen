"""
NF LI5640 ロックインアンプ（GPIB、読み取り専用。内部発振器だけは設定で許可したときに書く）

PyVISA は LI5640 を生成するときに初めて import する（Mock / テストでは不要）。
"""
from .li5640 import (READ_OUTPUT_TYPE, LI5640, LI5640Aborted, LI5640Error, LockinStepError,
                     OscillatorWriteError, describe, invalid_reason, theta_median)

__all__ = ["READ_OUTPUT_TYPE", "LI5640", "LI5640Aborted", "LI5640Error", "LockinStepError",
           "OscillatorWriteError", "describe", "invalid_reason", "theta_median"]
