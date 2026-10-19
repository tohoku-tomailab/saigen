"""
Agilent 1000 シリーズ（DSO1014A など）デジタルオシロスコープ（USB、読み取り専用）

PyVISA は DSO1000 を生成するときに初めて import する（Mock / テストでは不要）。
"""
from .dso1000 import (ITEMS, MODELS, DSO1000, DSO1000Aborted, DSO1000Error, ScopeStepError,
                      parse_measurement)

__all__ = ["ITEMS", "MODELS", "DSO1000", "DSO1000Aborted", "DSO1000Error", "ScopeStepError",
           "parse_measurement"]
