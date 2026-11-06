"""move_angle は r（= J1 + J4）を保つ。J4 を保つと、XYZ 移動（r を保つ）と
交互に使ったときに J4 が回転のたびにたまり、可動域を超えて alarm 0x22 になる
（2026-11 のチューブバランス運転: ホーム復帰のたびに J4 が約 30° 増えた）。"""
import math

import pytest

pytest.importorskip("pydobot")

from src.devices.dobot import pydobot_controller as pc
from src.devices.dobot.pydobot_controller import PyDobotController


class JointDevice:
    """J1/J4 と r の関係だけを再現する Dobot。"""

    def __init__(self, j1=-8.3, j4=0.0):
        self.x, self.y, self.z = 250.0, 0.0, 110.0
        self.j1, self.j4 = j1, j4
        self.ptp = []

    def pose(self):
        return (self.x, self.y, self.z, self.j1 + self.j4, self.j1, 25.0, 5.0, self.j4)

    def move_to(self, x, y, z, r, wait=True):
        # 直交移動: 指定の r になるように J4 が決まる
        self.x, self.y, self.z = x, y, z
        self.j1 = math.degrees(math.atan2(y, x))
        self.j4 = r - self.j1

    def _set_ptp_cmd(self, j1, j2, j3, j4, mode=None, wait=True):
        self.ptp.append((j1, j2, j3, j4, mode))
        self.j1, self.j4 = j1, j4


def _ctrl(dev):
    ctrl = object.__new__(PyDobotController)
    ctrl.port_name = "TEST"
    ctrl.device = dev
    ctrl._connected = True
    ctrl.verbose = False
    return ctrl


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(pc.time, "sleep", lambda s: None)


def test_move_angle_keeps_r():
    dev = JointDevice(j1=-38.8, j4=10.0)
    _ctrl(dev).move_angle(-8.3)
    j1, j2, j3, j4, mode = dev.ptp[-1]
    assert j1 == -8.3
    assert (j2, j3) == (25.0, 5.0)
    assert j1 + j4 == pytest.approx(-38.8 + 10.0)
    assert mode == pc.PTPMode.MOVJ_ANGLE


def test_j4_does_not_accumulate_over_balancing_cycles():
    """ホーム(J1 回転) → ビーカー → チューブ → ホーム を何度繰り返しても J4 は増えない。"""
    dev = JointDevice()
    ctrl = _ctrl(dev)
    beaker, tube = (239.3, 103.82), (207.95, -167.24)
    j4_seen = []
    for _ in range(20):
        ctrl.move_angle(-8.3)
        for x, y in (beaker, tube):
            ctrl.move_XYZ_abs(x, y, 110.9)
            j4_seen.append(dev.j4)
    assert max(abs(j) for j in j4_seen) < 45
    assert j4_seen[-1] == pytest.approx(j4_seen[1])
