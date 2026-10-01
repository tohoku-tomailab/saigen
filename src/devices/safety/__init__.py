"""
L1層: 安全ラッパ層

L0層のデバイスドライバを安全にラップし、
実験ワークフローで使いやすいインターフェースを提供します。

- LabRobot: ロボットアーム毎のデバイス管理（Dobot, Picus2等）
- MaxArmRobot / MockMaxArmRobot: Hiwonder MaxArm（吸着ノズル）の安全ラッパとモック
- SharedDevices: 共有デバイス管理（カメラ, 電子天秤等）
"""

from .lab_robot import LabRobot
from .maxarm_robot import MaxArmRobot
from .mock_maxarm import MockMaxArmRobot
from .shared_devices import SharedDevices

__all__ = ['LabRobot', 'MaxArmRobot', 'MockMaxArmRobot', 'SharedDevices']

__version__ = '1.0.0'
