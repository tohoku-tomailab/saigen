"""MaxArm 用の可動域バリデータ: 箱と半径に加えて、実機の逆運動学が解けるかを検証する

Hiwonder MaxArm は目標座標を ESP32 上のコンパイル済み逆運動学で解き、解けない
目標（関節角が 0..240° を外れる、リーチ外、hypot(x, y) < 50、z > 225）は黙って
拒否する。可動域の箱の中でも届かない座標があり（例: ホーミング後の高さ z≈222 は
箱の中だが逆運動学の範囲外）、``--validate-only`` の箱だけでは見つからない。

``MaxArmWorkspaceValidator`` は ``WorkspaceValidator`` の箱・半径の検証を先に行い、
そのうえで ``src.devices.maxarm.kinematics`` により同じ受理規則を PC 側で再現して、
届かない目標を ``WorkspaceViolationError`` として（コマンドを送る前に）拒否する。
締める方向にしか働かない: 箱・半径の判定は基底クラスのまま。
"""

import logging
from typing import Dict, List

from src.devices.maxarm import kinematics
from .exceptions import ViolationDetail, WorkspaceViolationError
from .workspace import WorkspaceValidator

logger = logging.getLogger(__name__)


class MaxArmWorkspaceValidator(WorkspaceValidator):
    """箱・半径（``WorkspaceValidator``）＋ MaxArm の逆運動学による到達判定"""

    def validate_xyz(self, x: float, y: float, z: float) -> None:
        """XYZ 座標の検証: 箱・半径のあとに逆運動学の受理判定

        Raises:
            WorkspaceViolationError: 箱・半径の外、または実機の逆運動学が拒否する座標
        """
        super().validate_xyz(x, y, z)
        err = kinematics.reachability(x, y, z)
        if err is None:
            logger.debug(f"MaxArm 到達判定OK: ({x:.2f}, {y:.2f}, {z:.2f})")
            return
        target: Dict[str, float] = {"x": x, "y": y, "z": z}
        hint = ""
        if err.axis.startswith("joint"):
            top = kinematics.highest_reachable_z(x, y, self.z_min, min(self.z_max, kinematics.Z_MAX))
            if top is not None:
                target["highest_reachable_z"] = top
                hint = f"（この (x, y) で届く最高の z は約 {top:.0f} mm）"
        violations: List[ViolationDetail] = [
            ViolationDetail(axis=err.axis, current_value=err.value,
                            min_value=err.low, max_value=err.high)
        ]
        raise WorkspaceViolationError(
            message=f"MaxArm の逆運動学では届かない座標です: {err.reason}{hint}",
            target_position=target,
            violations=violations,
        )

    def __repr__(self) -> str:
        return "MaxArm" + super().__repr__() + "[+IK]"
