"""
Keithley 2002 デジタルマルチメータ（GPIB / SCPI）

PyVISA は Keithley2002 を生成するときに初めて import する（Mock / テストでは不要）。
"""
from .keithley2002 import (FUNCTIONS, NPLC_MAX, NPLC_MIN, Keithley2002, Keithley2002Aborted,
                           Keithley2002Error)

__all__ = ["FUNCTIONS", "NPLC_MAX", "NPLC_MIN", "Keithley2002", "Keithley2002Aborted",
           "Keithley2002Error"]
