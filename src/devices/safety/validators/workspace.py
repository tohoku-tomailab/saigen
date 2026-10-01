"""
可動域バリデータ

X, Y, Z座標とJoint1角度の可動域を検証します。
任意で水平半径 r = hypot(X, Y) の範囲（r_min / r_max）も検証できます
（MaxArm など、基部近傍や最大到達距離付近を避けたいアーム用）。
"""

import logging
import math
from typing import Dict, Optional, List

from .base import PositionValidator
from .exceptions import WorkspaceViolationError, ViolationDetail

logger = logging.getLogger(__name__)


class NonFiniteTargetError(WorkspaceViolationError, ValueError):
    """目標値に NaN / inf が含まれる場合の可動域違反。

    NaN は大小比較が常に False になるため、範囲チェックだけでは素通りしてしまう。
    WorkspaceViolationError と ValueError の両方として捕捉できる。
    """

    def __str__(self) -> str:
        pos_str = ", ".join(f"{k}={v}" for k, v in self.target_position.items())
        axes = ", ".join(v.axis for v in self.violations)
        return (
            f"{self.message}\n"
            f"目標位置: ({pos_str})\n"
            f"違反内容:\n  - 有限の数値ではない軸: {axes}"
        )


def _non_finite(values: Dict[str, float]) -> List[str]:
    """有限の実数でない（NaN / inf / 数値でない）キーを返す。"""
    bad = []
    for key, value in values.items():
        try:
            if not math.isfinite(value):
                bad.append(key)
        except TypeError:
            bad.append(key)
    return bad



class WorkspaceValidator(PositionValidator):
    """
    直交座標系の可動域バリデータ

    X, Y, Z各軸の最小・最大値とJoint1角度の範囲を検証します。

    Args:
        x_min, x_max: X軸の可動範囲 (mm)
        y_min, y_max: Y軸の可動範囲 (mm)
        z_min, z_max: Z軸の可動範囲 (mm)
        joint1_min, joint1_max: Joint1の可動範囲 (度)
        r_min, r_max: 水平半径 hypot(X, Y) の可動範囲 (mm)。任意（None = 検証しない）。
            validate_xyz でのみ検証する。既定値は None で、既存の検証は変わらない

    使用例:
        validator = WorkspaceValidator(
            x_min=-300, x_max=300,
            y_min=-300, y_max=300,
            z_min=-130, z_max=150,
            joint1_min=-135, joint1_max=135
        )

        # 検証（可動域外なら例外）
        validator.validate_xyz(x=200, y=100, z=50)
        validator.validate_joint1(angle=45)
    """

    def __init__(
        self,
        x_min: float = -300,
        x_max: float = 300,
        y_min: float = -300,
        y_max: float = 300,
        z_min: float = -130,
        z_max: float = 150,
        joint1_min: float = -135,
        joint1_max: float = 135,
        r_min: Optional[float] = None,
        r_max: Optional[float] = None,
    ):
        limits = {
            "x": (x_min, x_max),
            "y": (y_min, y_max),
            "z": (z_min, z_max),
            "joint1": (joint1_min, joint1_max),
        }
        for axis, (lo, hi) in limits.items():
            bad = _non_finite({f"{axis}_min": lo, f"{axis}_max": hi})
            if bad:
                raise ValueError(
                    f"可動域の上下限が有限の数値ではありません: "
                    f"{axis}_min={lo!r}, {axis}_max={hi!r}"
                )
            if lo > hi:
                raise ValueError(
                    f"可動域の下限が上限より大きい: {axis}_min={lo} > {axis}_max={hi}"
                )

        for name, value in (("r_min", r_min), ("r_max", r_max)):
            if value is None:
                continue
            if _non_finite({name: value}):
                raise ValueError(f"可動域の半径制限が有限の数値ではありません: {name}={value!r}")
            if value < 0:
                raise ValueError(f"可動域の半径制限は 0 以上で指定してください: {name}={value}")
        if r_min is not None and r_max is not None and r_min > r_max:
            raise ValueError(
                f"可動域の下限が上限より大きい: r_min={r_min} > r_max={r_max}"
            )

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        self.joint1_min = joint1_min
        self.joint1_max = joint1_max
        self.r_min = r_min
        self.r_max = r_max

        logger.debug(f"WorkspaceValidator初期化: {self!r}")

    @property
    def has_radius_limits(self) -> bool:
        """r_min / r_max のどちらかが設定されているか"""
        return self.r_min is not None or self.r_max is not None

    def validate_xyz(self, x: float, y: float, z: float) -> None:
        """
        XYZ座標の検証

        Args:
            x: X座標 (mm)
            y: Y座標 (mm)
            z: Z座標 (mm)

        Raises:
            WorkspaceViolationError: 可動域外の場合（r_min / r_max 設定時は
                水平半径 hypot(x, y) の範囲外も含む）
        """
        violations: List[ViolationDetail] = []
        self._require_finite({"x": x, "y": y, "z": z}, "XYZ座標が有限の数値ではありません (NaN/inf)")
        target = {"x": x, "y": y, "z": z}

        # X軸チェック
        if x < self.x_min or x > self.x_max:
            violations.append(
                ViolationDetail(
                    axis="X",
                    current_value=x,
                    min_value=self.x_min,
                    max_value=self.x_max,
                )
            )

        # Y軸チェック
        if y < self.y_min or y > self.y_max:
            violations.append(
                ViolationDetail(
                    axis="Y",
                    current_value=y,
                    min_value=self.y_min,
                    max_value=self.y_max,
                )
            )

        # Z軸チェック
        if z < self.z_min or z > self.z_max:
            violations.append(
                ViolationDetail(
                    axis="Z",
                    current_value=z,
                    min_value=self.z_min,
                    max_value=self.z_max,
                )
            )

        # 水平半径チェック（設定時のみ）
        if self.has_radius_limits:
            r = math.hypot(x, y)
            target["r"] = r
            too_close = self.r_min is not None and r < self.r_min
            too_far = self.r_max is not None and r > self.r_max
            if too_close or too_far:
                violations.append(
                    ViolationDetail(
                        axis="R",
                        current_value=r,
                        min_value=self.r_min if self.r_min is not None else 0.0,
                        max_value=self.r_max if self.r_max is not None else math.inf,
                    )
                )

        if violations:
            raise WorkspaceViolationError(
                message="XYZ座標が可動域外です",
                target_position=target,
                violations=violations,
            )

        logger.debug(f"XYZ座標検証OK: ({x:.2f}, {y:.2f}, {z:.2f})")

    def validate_joint1(self, angle: float) -> None:
        """
        Joint1角度の検証

        Args:
            angle: Joint1角度 (度)

        Raises:
            WorkspaceViolationError: 可動域外の場合
        """
        self._require_finite({"joint1": angle}, "Joint1角度が有限の数値ではありません (NaN/inf)")
        if angle < self.joint1_min or angle > self.joint1_max:
            raise WorkspaceViolationError(
                message="Joint1角度が可動域外です",
                target_position={"joint1": angle},
                violations=[
                    ViolationDetail(
                        axis="Joint1",
                        current_value=angle,
                        min_value=self.joint1_min,
                        max_value=self.joint1_max,
                    )
                ],
            )

        logger.debug(f"Joint1角度検証OK: {angle:.2f}°")

    def validate_z_relative(self, current_z: float, delta_z: float) -> None:
        """
        相対Z移動の検証

        Args:
            current_z: 現在のZ座標 (mm)
            delta_z: 移動量 (mm、正=上昇、負=下降)

        Raises:
            WorkspaceViolationError: 移動後が可動域外の場合
        """
        self._require_finite({"current_z": current_z, "delta_z": delta_z}, "Z軸相対移動の値が有限の数値ではありません (NaN/inf)")
        target_z = current_z + delta_z

        if target_z < self.z_min or target_z > self.z_max:
            raise WorkspaceViolationError(
                message="Z軸移動後の位置が可動域外です",
                target_position={"current_z": current_z, "delta_z": delta_z, "target_z": target_z},
                violations=[
                    ViolationDetail(
                        axis="Z",
                        current_value=target_z,
                        min_value=self.z_min,
                        max_value=self.z_max,
                    )
                ],
            )

        logger.debug(f"Z相対移動検証OK: {current_z:.2f} + {delta_z:.2f} = {target_z:.2f}")

    def validate_joint1_relative(self, current_angle: float, delta_angle: float) -> None:
        """
        相対Joint1回転の検証

        Args:
            current_angle: 現在のJoint1角度 (度)
            delta_angle: 回転量 (度)

        Raises:
            WorkspaceViolationError: 回転後が可動域外の場合
        """
        self._require_finite({"current_angle": current_angle, "delta_angle": delta_angle}, "Joint1相対回転の値が有限の数値ではありません (NaN/inf)")
        target_angle = current_angle + delta_angle

        if target_angle < self.joint1_min or target_angle > self.joint1_max:
            raise WorkspaceViolationError(
                message="Joint1回転後の角度が可動域外です",
                target_position={
                    "current_angle": current_angle,
                    "delta_angle": delta_angle,
                    "target_angle": target_angle,
                },
                violations=[
                    ViolationDetail(
                        axis="Joint1",
                        current_value=target_angle,
                        min_value=self.joint1_min,
                        max_value=self.joint1_max,
                    )
                ],
            )

        logger.debug(
            f"Joint1相対回転検証OK: {current_angle:.2f}° + {delta_angle:.2f}° = {target_angle:.2f}°"
        )

    @staticmethod
    def _require_finite(values: Dict[str, float], message: str) -> None:
        """NaN / inf を含む目標値を拒否する（NaN は範囲比較をすり抜けるため）。"""
        bad = _non_finite(values)
        if bad:
            raise NonFiniteTargetError(
                message=message,
                target_position=dict(values),
                violations=[
                    ViolationDetail(axis=k, current_value=values[k],
                                    min_value=float("nan"), max_value=float("nan"))
                    for k in bad
                ],
            )

    def get_limits(self) -> Dict[str, Dict[str, float]]:
        """現在の可動域制限を取得

        r_min / r_max は設定されている場合のみ ``"r"`` キーに含める
        （設定されている側の ``"min"`` / ``"max"`` だけを持つ）。
        """
        limits = {
            "x": {"min": self.x_min, "max": self.x_max},
            "y": {"min": self.y_min, "max": self.y_max},
            "z": {"min": self.z_min, "max": self.z_max},
            "joint1": {"min": self.joint1_min, "max": self.joint1_max},
        }
        if self.has_radius_limits:
            r = {}
            if self.r_min is not None:
                r["min"] = self.r_min
            if self.r_max is not None:
                r["max"] = self.r_max
            limits["r"] = r
        return limits

    def __repr__(self) -> str:
        return (
            f"WorkspaceValidator("
            f"X=[{self.x_min}, {self.x_max}], "
            f"Y=[{self.y_min}, {self.y_max}], "
            f"Z=[{self.z_min}, {self.z_max}], "
            f"Joint1=[{self.joint1_min}, {self.joint1_max}]"
            + (f", R=[{self.r_min}, {self.r_max}]" if self.has_radius_limits else "")
            + ")"
        )
