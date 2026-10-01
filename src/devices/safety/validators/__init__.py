"""
L1層: バリデータモジュール

動作前検証を行うバリデータを提供します。
拡張可能な設計により、新しい検証ロジックを容易に追加できます。

使用例:
    from src.devices.safety.validators import WorkspaceValidator, WorkspaceViolationError

    validator = WorkspaceValidator(
        x_min=-300, x_max=300,
        y_min=-300, y_max=300,
        z_min=-130, z_max=150,
        joint1_min=-135, joint1_max=135
    )
    validator.validate_xyz(x=200, y=100, z=50)  # 可動域内ならOK
    validator.validate_joint1(angle=45)  # 可動域内ならOK

    # 任意の水平半径制限（r = hypot(x, y)。MaxArm などで使う）
    validator = WorkspaceValidator(z_min=40, z_max=225, r_min=60, r_max=260)

    # config 形式の辞書（共通 + ロボット別の上書き）から生成
    validator = workspace_validator_from_config(
        {"x_min": -250, "x_max": 250}, {"r_min": 60, "r_max": 260})
"""

import logging
from typing import Any, Dict, Mapping, Optional, Type

from .base import PositionValidator, ValidationResult
from .exceptions import (
    ValidationError,
    WorkspaceViolationError,
    ViolationDetail,
)
from .workspace import WorkspaceValidator

_logger = logging.getLogger(__name__)


def default_workspace_validator(**overrides) -> WorkspaceValidator:
    """既定の安全可動域で ``WorkspaceValidator`` を生成するファクトリ。

    可動域の値はリポジトリ直下 config.yaml の ``workspace`` セクション
    （src.config 経由）を単一ソースとする。src.config を import できない
    単体利用時は :class:`WorkspaceValidator` の既定（config.example.yaml と
    同値）にフォールバックする。呼び出し側は ``overrides`` で個別に
    上書きできる。

    Args:
        **overrides: WorkspaceValidator のコンストラクタ引数の上書き
            （例: ``z_min=-100``）

    Returns:
        WorkspaceValidator: 可動域バリデータ
    """
    params = {}
    try:
        from src import config as lab_config
        params.update(lab_config.get_workspace())
    except ImportError:
        # リポジトリルートが sys.path に無い単体利用時はクラス既定値で動作
        pass
    params.update(overrides)
    return WorkspaceValidator(**params)


#: WorkspaceValidator のコンストラクタが受け付ける可動域キー
WORKSPACE_KEYS = (
    "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
    "joint1_min", "joint1_max", "r_min", "r_max",
)


def workspace_validator_from_config(base: Optional[Mapping[str, Any]],
                                    override: Optional[Mapping[str, Any]] = None,
                                    cls: Type[WorkspaceValidator] = WorkspaceValidator
                                    ) -> WorkspaceValidator:
    """可動域の辞書（config.yaml の ``workspace`` 形式）から ``WorkspaceValidator`` を作る

    ``base``（全体の ``workspace``）に ``override``（ロボットごとの
    ``robots.<id>.workspace``）を上書きマージする。純粋関数で、config は読まない。
    未知のキーは警告を出して無視する。

    値が None（YAML で ``z_min:`` のように値を書き忘れた場合など）のキーは、
    base / override のどちらにあっても ValueError にする。None を「未指定」として
    読み飛ばすと、測った制限値（例: ``z_min: 40``）が WorkspaceValidator の既定値
    （Dobot 用の広い箱）に戻ったり、r_min / r_max の制限が黙って外れたりするため。
    半径の制限を使わない場合は、キーごと書かない。

    Args:
        base: 共通の可動域（x_min .. joint1_max, 任意で r_min / r_max）
        override: ロボット固有の可動域（base のキーを上書き）

    Returns:
        WorkspaceValidator: 可動域バリデータ

    Raises:
        ValueError: 値が None のキーがある場合、または上下限が不正
            （有限でない、下限 > 上限など）な場合
    """
    params: Dict[str, Any] = {}
    for label, source in (("workspace", base), ("robot workspace", override)):
        if not source:
            continue
        for key, value in dict(source).items():
            if key not in WORKSPACE_KEYS:
                _logger.warning(f"workspace の未知のキーを無視します: {key}={value!r}")
                continue
            if value is None:
                raise ValueError(
                    f"workspace.{key} is null ({label}): 値を書くか、キーごと削除してください"
                    "（null は既定値への置き換えや制限の解除にはなりません）"
                )
            params[key] = value
    return cls(**params)


def maxarm_workspace_validator_from_config(workspace: Mapping[str, Any]) -> "MaxArmWorkspaceValidator":
    """MaxArm の ``robots.<id>.workspace``（完全な箱＋任意の半径）から、箱・半径に
    加えて実機の逆運動学による到達判定を行う ``MaxArmWorkspaceValidator`` を作る。
    値の検査は :func:`workspace_validator_from_config` と同じ（None は ValueError）。"""
    from .maxarm import MaxArmWorkspaceValidator  # 遅延 import（kinematics を引き込む）
    return workspace_validator_from_config(None, workspace, cls=MaxArmWorkspaceValidator)


__all__ = [
    # 基底クラス
    "PositionValidator",
    "ValidationResult",
    # 例外
    "ValidationError",
    "WorkspaceViolationError",
    "ViolationDetail",
    # バリデータ
    "WorkspaceValidator",
    # ファクトリ
    "default_workspace_validator",
    "workspace_validator_from_config",
    "maxarm_workspace_validator_from_config",
    "WORKSPACE_KEYS",
]
