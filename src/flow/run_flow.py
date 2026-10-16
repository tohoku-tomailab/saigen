"""
run_flow.py - JSON 実験フローをコマンドラインから実行する

使い方:
    # 実機なしで動作確認（Mock モード。録画は常に無効）
    python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --mock

    # スキーマ検証・ループ展開・絶対目標の可動域チェックだけ行う（デバイスは開かない）
    python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --validate-only

    # 実機で実行（ポートは config.yaml / .env / 引数で指定）
    python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json \
        --robot1-dobot COM3 --robot1-picus2 COM4 \
        --robot2-dobot COM5 --robot2-picus2 COM6 \
        --scale-port COM8

    # Keithley 2002 DMM（GPIB）の VISA リソースは --dmm-resource
    # NF LI5640 ロックインアンプ（GPIB）の VISA リソースは --lockin-resource
    python -m src.flow.run_flow examples/keithley2002/measure_dc_voltage.json --mock

    # MaxArm（robot 4, config.yaml の type: maxarm）のポートは --robot4-maxarm
    python -m src.flow.run_flow examples/maxarm/pick_and_place.json --mock

処理の流れ:
    1. JSON を読み込み、Pydantic スキーマ（schema.py）で検証する
    2. loop_start / loop_end を展開してフラットなステップ列にする
    2.5 プリフライト: 絶対目標（move_xyz / rotate）を可動域バリデータで検査する。
       可動域はロボットごと（robots.<id>.workspace があればそれ、無ければ共通の
       workspace）。各アクションがそのロボットの種別（dobot / maxarm）で
       実行できるかも検査する。違反があればデバイスを開く前に中止する
       （--validate-only でも同じ）。
       相対移動（move_z / rotate_relative / move_radial）は開始姿勢に依存するため
       ここでは検査できず、--mock / 実機実行時に 1 手ごとに検査される
    3. 実行ごとのログフォルダを作る（logs/<日付>/<フロー名>_<時刻>/）
    4. ExperimentSession の安全枠（初期化 → 実行 → 中断時は緊急停止 /
       エラー時はホーム復帰 → 切断）の中でステップを順に実行する
    5. 各ステップの結果（重量・画像パス・DMM 測定値）を measurements.csv / summary.md に残し、
       dispense と直後の measure_weight を対応させた分注精度 CSV も出力する

接続ポートの優先順位:
    コマンドライン引数 > 環境変数（.env / ROBOT1_DOBOT_PORT など）
    > config.yaml（無ければ config.example.yaml）> 組み込みデフォルト
"""
import argparse
import asyncio
import json
import logging
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple, Optional, Tuple
from urllib import request as urlrequest
from urllib.error import URLError

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from dotenv import load_dotenv
from pydantic import ValidationError

from src import config as lab_config
from src.devices.safety.validators import WorkspaceViolationError, default_workspace_validator
from src.flow.accuracy_logger import DispenseAccuracyLogger
from src.flow.executor import execute_step, expand_loops, SHARED_DEVICE_ACTIONS, MICROSCOPE_CAMERA_ACTIONS, MICROSCOPE_SERIAL_ACTIONS, DMM_ACTIONS, LOCKIN_ACTIONS
from src.flow.experiment_logger import ExperimentLogger
from src.flow.experiment_session import ExperimentSession, robot_type, validator_for_robot
from src.flow.schema import MAX_ROBOT_ID, ExperimentWorkflow

load_dotenv()
logger = logging.getLogger("run_flow")

PICUS2_ACTIONS = {"aspirate", "dispense", "blow_out"}
LOOP_ACTIONS = {"loop_start", "loop_end"}
#: ロボット種別ごとに実行できるロボットアクション（共有デバイス・ループは対象外）。
#: preflight_capabilities が実行前に、executor が実行時（SUPPORTED_ACTIONS）に検査する。
ROBOT_ACTIONS_BY_TYPE = {
    "dobot": frozenset({"move_xyz", "move_z", "move_radial", "rotate", "rotate_relative",
                        "go_home", "move_slider", "move_conveyer", "wait",
                        "aspirate", "dispense", "blow_out"}),
    "maxarm": frozenset({"move_xyz", "move_z", "go_home", "suction", "wait"}),
}
# 開始姿勢に依存し、実行前には目標位置が決まらない相対移動
RELATIVE_MOVE_ACTIONS = {"move_z", "rotate_relative", "move_radial"}
LOGS_DIR = os.path.join(_REPO_ROOT, "logs")
DASHBOARD_SCRIPT = os.path.join(
    _REPO_ROOT, "src", "monitoring", "dashboard", "launch_sensor_dashboard.py"
)


# ======================================================================
# センサーダッシュボード（CSV 録画）との連携
# ======================================================================
def _sensor_server_url() -> str:
    """ダッシュボードの URL。SENSOR_SERVER_URL > config の web_port の順。"""
    env_url = os.environ.get("SENSOR_SERVER_URL")
    if env_url:
        return env_url.rstrip("/")
    return f"http://localhost:{lab_config.get_dashboard_ports()['web_port']}"


def _auth_headers() -> dict:
    token = os.environ.get("SENSOR_DASHBOARD_TOKEN")
    return {"X-Auth-Token": token} if token else {}


def _http_get_json(url: str, timeout: float = 2.0):
    req = urlrequest.Request(url, headers=_auth_headers(), method="GET")
    with urlrequest.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _http_post_json(url: str, body: dict, timeout: float = 5.0) -> Tuple[int, dict]:
    data = json.dumps(body).encode("utf-8")
    req = urlrequest.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json", **_auth_headers()},
        method="POST",
    )
    try:
        with urlrequest.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8") or "{}")
    except URLError as e:
        # サーバはあるがステータスが 2xx 以外（409 など）の場合もここに来る
        status = getattr(e, "code", 0)
        try:
            body_text = e.read().decode("utf-8")  # type: ignore[attr-defined]
        except Exception:
            body_text = ""
        return status, {"detail": body_text}


def ensure_dashboard_running(timeout: float = 15.0) -> bool:
    """センサーダッシュボードが起動していなければ headless で立ち上げる。"""
    base = _sensor_server_url()
    try:
        _http_get_json(f"{base}/api/status", timeout=1.5)
        logger.info("センサーダッシュボードは既に起動しています")
        return True
    except (URLError, ConnectionError, TimeoutError, OSError):
        pass

    if not os.path.exists(DASHBOARD_SCRIPT):
        logger.warning(f"ダッシュボードスクリプトが見つかりません: {DASHBOARD_SCRIPT}")
        return False

    logger.info("センサーダッシュボードを headless で起動します")
    env = os.environ.copy()
    env["SENSOR_DASHBOARD_HEADLESS"] = "1"
    creationflags = 0
    if os.name == "nt":
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    subprocess.Popen(
        [sys.executable, DASHBOARD_SCRIPT],
        cwd=os.path.dirname(DASHBOARD_SCRIPT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=creationflags,
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            _http_get_json(f"{base}/api/status", timeout=1.0)
            logger.info("センサーダッシュボード起動完了")
            return True
        except Exception:
            time.sleep(0.5)
    logger.warning("センサーダッシュボードの起動確認がタイムアウトしました")
    return False


@dataclass
class RecordingHandle:
    """このランが開始した（= 止めてよい）センサー CSV 録画かどうか。

    ``owned`` が True になるのは /api/start が 200 を返したときだけ。409（既に
    録画中）は「他の誰かの録画」なので、このランは /api/end を送らない。
    """
    owned: bool = False
    save_dir: Optional[str] = None
    status: int = 0

    def as_metadata(self) -> dict:
        return {"started_by_this_run": self.owned, "save_dir": self.save_dir,
                "start_status": self.status}


def start_csv_recording(experiment_name: str) -> RecordingHandle:
    """/api/start を叩いて CSV 録画を開始する。

    Returns:
        RecordingHandle。200 のときだけ ``owned=True``。409 は既に別の録画が
        進行中という意味で、保存先は参考として取得するが ``owned=False``。
    """
    base = _sensor_server_url()
    status, body = _http_post_json(f"{base}/api/start", {"experiment_name": experiment_name})
    if status == 200:
        save_dir = body.get("save_dir")
        logger.info(f"CSV録画開始 (save_dir={save_dir})")
        return RecordingHandle(owned=True, save_dir=save_dir, status=status)
    if status == 409:
        logger.info("CSV録画開始スキップ: 既に録画中（このランが開始した録画ではないため、"
                    "終了時にも止めません）")
        try:
            save_dir = _http_get_json(f"{base}/api/status").get("save_dir")
        except Exception:
            save_dir = None
        return RecordingHandle(owned=False, save_dir=save_dir, status=status)
    logger.warning(f"CSV録画開始失敗: {status} {body}")
    return RecordingHandle(owned=False, save_dir=None, status=status)


def stop_csv_recording(handle: Optional[RecordingHandle]) -> bool:
    """このランが開始した録画だけを /api/end で止める。

    Returns:
        /api/end を送ったら True（``handle`` が None / 他人の録画なら送らず False）
    """
    if handle is None or not handle.owned:
        logger.info("CSV録画終了スキップ: このランが開始した録画ではありません")
        return False
    status, body = _http_post_json(f"{_sensor_server_url()}/api/end", {})
    if status == 200:
        logger.info(f"CSV録画終了 (saved_path={body.get('saved_path')})")
    elif status == 409:
        logger.info("CSV録画終了スキップ: 録画していません")
    else:
        logger.warning(f"CSV録画終了失敗: {status} {body}")
    return True


# ======================================================================
# CLI
# ======================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="JSON 実験フローを実行する")
    p.add_argument("flow", help="実行する JSON フローのパス")
    p.add_argument("--mock", action="store_true",
                   help="実機を使わずログ出力のみで実行する（録画は常に無効）")
    p.add_argument("--validate-only", action="store_true",
                   help="スキーマ検証・ループ展開・絶対目標（move_xyz / rotate）の可動域チェックのみ行い、"
                        "デバイスを開かずに終了する。相対移動は --mock / 実行時に 1 手ごとに検査される")
    # ポートは None を既定にして、環境変数 → config.yaml の順で解決する
    for rid in range(1, MAX_ROBOT_ID + 1):
        p.add_argument(f"--robot{rid}-dobot", default=None,
                       help=f"Robot {rid} の Dobot ポート"
                            f"（既定: 環境変数 ROBOT{rid}_DOBOT_PORT / config.yaml）")
        p.add_argument(f"--robot{rid}-picus2", default=None,
                       help=f"Robot {rid} の Picus2 ポート。空なら Picus2 なし"
                            f"（既定: ROBOT{rid}_PICUS2_PORT / config.yaml）")
        p.add_argument(f"--robot{rid}-maxarm", default=None,
                       help=f"Robot {rid} が MaxArm（config.yaml の type: maxarm）の場合のポート"
                            f"（既定: ROBOT{rid}_MAXARM_PORT / config.yaml）")
    p.add_argument("--scale-port", default=None,
                   help="電子天秤のポート（既定: SCALE_PORT / config.yaml）")
    p.add_argument("--camera-index", type=int, default=None,
                   help="capture_and_save 用カメラ index（既定: CAMERA_INDEX / config.yaml）")
    p.add_argument("--microscope-index", type=int, default=None,
                   help="capture_microscope 用の顕微鏡カメラ index（既定: MICROSCOPE_INDEX / config.yaml）")
    p.add_argument("--microscope-port", default=None,
                   help="顕微鏡 LED 制御用シリアルポート（既定: MICROSCOPE_PORT / config.yaml。空なら LED 制御なし）")
    p.add_argument("--dmm-resource", default=None,
                   help="Keithley 2002 DMM の VISA リソース（例 GPIB0::16::INSTR。"
                        "既定: DMM_RESOURCE / config.yaml）")
    p.add_argument("--lockin-resource", default=None,
                   help="NF LI5640 ロックインアンプの VISA リソース（例 GPIB0::<アドレス>::INSTR。"
                        "既定: LOCKIN_RESOURCE / config.yaml）")
    record = p.add_mutually_exclusive_group()
    record.add_argument("--record", dest="record", action="store_true", default=None,
                        help="センサーCSV・動画の録画を行う（実機実行時の既定）")
    record.add_argument("--no-record", dest="record", action="store_false",
                        help="録画を行わない")
    p.add_argument("--liquid-density", type=float, default=1.0,
                   help="分注精度計算に使う液体密度 g/mL（既定: 1.0 = 水）")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def resolve_ports(args) -> Tuple[dict, dict]:
    """接続設定を「引数 > 環境変数 > config.yaml」の優先順で解決する。

    ロボットの種別（``type``）・ロボット別の ``workspace``・ドライバオプションは
    config.yaml の値をそのまま保つ（上書きできるのはポートだけ）。

    Returns:
        (robot_ports, shared_config)
    """
    robot_ports = lab_config.get_robot_ports()
    shared_config = lab_config.get_shared_devices()

    for rid in range(1, MAX_ROBOT_ID + 1):
        ports = dict(robot_ports.get(rid, {}))
        ports.setdefault("type", lab_config.DEFAULT_ROBOT_TYPE)
        dobot = getattr(args, f"robot{rid}_dobot", None) or os.getenv(f"ROBOT{rid}_DOBOT_PORT")
        picus2 = getattr(args, f"robot{rid}_picus2", None) or os.getenv(f"ROBOT{rid}_PICUS2_PORT")
        maxarm = getattr(args, f"robot{rid}_maxarm", None) or os.getenv(f"ROBOT{rid}_MAXARM_PORT")
        if dobot:
            ports["dobot_port"] = dobot
        if picus2:
            ports["picus2_address"] = picus2
        if maxarm:
            if ports["type"] != "maxarm":
                logger.warning(f"Robot {rid} は type={ports['type']} のため MaxArm ポート "
                               f"{maxarm} は使われません（config.yaml の robots.{rid}.type を確認）")
            ports["maxarm_port"] = maxarm
        robot_ports[rid] = ports

    scale = args.scale_port or os.getenv("SCALE_PORT")
    if scale:
        shared_config["scale_port"] = scale
    camera = args.camera_index if args.camera_index is not None else os.getenv("CAMERA_INDEX")
    if camera not in (None, ""):
        shared_config["camera_index"] = int(camera)
    microscope = (getattr(args, "microscope_index", None)
                  if getattr(args, "microscope_index", None) is not None
                  else os.getenv("MICROSCOPE_INDEX"))
    if microscope not in (None, ""):
        shared_config["microscope_index"] = int(microscope)
    cli_port = getattr(args, "microscope_port", None)
    if cli_port is not None:                       # "" は「制御ポートを使わない」の明示
        shared_config["microscope_port"] = cli_port
    elif os.getenv("MICROSCOPE_PORT"):
        shared_config["microscope_port"] = os.getenv("MICROSCOPE_PORT")
    # CSV ランナーは自前のパーサーでこの関数を呼ぶので getattr で読む
    dmm = getattr(args, "dmm_resource", None) or os.getenv("DMM_RESOURCE")
    if dmm:
        shared_config["dmm_resource"] = dmm
    lockin = getattr(args, "lockin_resource", None) or os.getenv("LOCKIN_RESOURCE")
    if lockin:
        shared_config["lockin_resource"] = lockin

    return robot_ports, shared_config


def load_and_validate(path: str) -> Tuple[ExperimentWorkflow, list]:
    """JSON を読み込み、スキーマ検証とループ展開を行う。

    Returns:
        (workflow, 展開後のステップ dict のリスト)
    """
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    workflow = ExperimentWorkflow(**data)  # 不正なフローはここで ValidationError
    logger.info(f"フロー: {workflow.name} / {workflow.description}")
    steps = [s.model_dump() for s in workflow.steps]
    expanded = expand_loops(steps)
    logger.info(f"ステップ数: {len(steps)}（ループ展開後 {len(expanded)}）")
    return workflow, expanded


@dataclass
class PreflightReport:
    """実行前の可動域・対応アクションチェックの結果（ステップ番号はループ展開後の 1 始まり）

    Attributes:
        capability_errors: ロボット種別が対応しないアクションなど（1 件 1 文）
        validators_by_robot: run_preflight が使ったロボットごとのバリデータ。
            ExperimentSession(robot_validators=...) に渡して実行時も同じ可動域を使う
        robot_types: フローが使うロボットの robot_id → 種別
    """
    checked: int = 0
    violations: List[Tuple[int, dict, Exception]] = field(default_factory=list)
    unverified: List[Tuple[int, dict]] = field(default_factory=list)
    capability_errors: List[str] = field(default_factory=list)
    validators_by_robot: Dict[int, object] = field(default_factory=dict)
    robot_types: Dict[int, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.violations and not self.capability_errors


def _step_label(index: int, step: dict) -> str:
    rid = step.get("robot_id", 1)
    it = step.get("_iteration")
    return f"ステップ {index}" + (f"（ループ {it} 回目）" if it else "") + f" Robot {rid} {step.get('action')}"


def preflight_workspace(steps: list, validator=None,
                        validators_by_robot: Optional[dict] = None) -> PreflightReport:
    """展開後のステップの絶対目標を、デバイスを開かずに可動域バリデータで検査する。

    ``validators_by_robot`` に robot_id があるステップはそのバリデータで、
    無いステップは ``validator``（共通の可動域）で検査する。

    - move_xyz: ``validate_xyz(x, y, z)``
    - rotate: ``validate_joint1(angle)``
    - move_z / rotate_relative / move_radial: 開始姿勢が実行前には分からないため
      検査せず ``unverified`` に入れる（実行時に LabRobot / MockLabRobot が 1 手
      ごとに検査する）

    NaN / inf もここで違反になる（WorkspaceValidator が拒否する）。
    """
    if validator is None:
        validator = default_workspace_validator()
    validators_by_robot = validators_by_robot or {}
    report = PreflightReport()
    for i, step in enumerate(steps, 1):
        action = step.get("action")
        v = validators_by_robot.get(step.get("robot_id", 1), validator)
        try:
            if action == "move_xyz":
                report.checked += 1
                v.validate_xyz(step.get("x"), step.get("y"), step.get("z"))
            elif action == "rotate":
                report.checked += 1
                v.validate_joint1(step.get("angle"))
            elif action in RELATIVE_MOVE_ACTIONS:
                report.unverified.append((i, step))
        except WorkspaceViolationError as e:
            report.violations.append((i, step, e))
    return report


def _robot_ids_in(steps: list) -> list:
    """ロボットアクション（共有デバイス・ループ以外）が使う robot_id"""
    return sorted({s.get("robot_id", 1) for s in steps
                   if s.get("action") not in SHARED_DEVICE_ACTIONS
                   and s.get("action") not in LOOP_ACTIONS})


def preflight_capabilities(steps: list, robot_types: dict) -> List[str]:
    """各ロボットアクションが、そのロボットの種別で実行できるかを検査する。

    例: MaxArm（type: maxarm）への aspirate、Dobot への suction、未知の種別。
    デバイスは開かない。同じ (robot, action) の誤りはまとめて 1 件にする。

    Args:
        steps: ループ展開後のステップ dict のリスト
        robot_types: robot_id → 種別（無い robot_id は "dobot"）

    Returns:
        エラー文のリスト（空なら OK）
    """
    errors = []
    seen = {}
    for i, step in enumerate(steps, 1):
        action = step.get("action")
        if action in SHARED_DEVICE_ACTIONS or action in LOOP_ACTIONS:
            continue
        rid = step.get("robot_id", 1)
        rtype = (robot_types or {}).get(rid) or lab_config.DEFAULT_ROBOT_TYPE
        allowed = ROBOT_ACTIONS_BY_TYPE.get(rtype)
        if allowed is not None and action in allowed:
            continue
        seen.setdefault((rid, rtype, action), []).append(i)
    for (rid, rtype, action), idx in seen.items():
        shown = ", ".join(map(str, idx[:10])) + (" ..." if len(idx) > 10 else "")
        allowed = ROBOT_ACTIONS_BY_TYPE.get(rtype)
        if allowed is None:
            errors.append(f"Robot {rid} の種別 {rtype!r} は未知です（config.yaml の "
                          f"robots.{rid}.type は {', '.join(ROBOT_ACTIONS_BY_TYPE)} のいずれか）"
                          f": ステップ {shown}")
        else:
            note = "（MaxArm にピペットはありません）" if (
                rtype == "maxarm" and action in PICUS2_ACTIONS) else ""
            errors.append(f"Robot {rid}（{rtype}）はアクション {action!r} に対応していません"
                          f"{note}: ステップ {shown}（{len(idx)} 回）。"
                          f"対応: {', '.join(sorted(allowed))}")
    return errors


def log_preflight(report: PreflightReport, validator, log=None) -> None:
    """プリフライト結果をログに出す（違反は ERROR、相対移動は INFO で要約）"""
    logger = log or globals()["logger"]
    for msg in report.capability_errors:
        logger.error(f"対応していないアクション: {msg}")
    for i, step, err in report.violations:
        logger.error(f"可動域違反 {_step_label(i, step)} {_fmt_params(step)}\n{err}")
    if report.violations:
        violating = {step.get("robot_id", 1) for _, step, _ in report.violations}
        own = {rid for rid, v in (report.validators_by_robot or {}).items()
               if v is not validator and rid in violating}
        if validator is not None and violating - own:
            logger.error(f"可動域の上下限（config.yaml の workspace）: {validator.get_limits()}")
        for rid in sorted(own):
            logger.error(f"Robot {rid} の可動域の上下限（config.yaml の robots.{rid}.workspace）: "
                         f"{report.validators_by_robot[rid].get_limits()}")

    if report.unverified:
        # ループ展開で同じステップが何度も出るので、同一内容ごとにまとめる
        groups = {}
        for i, step in report.unverified:
            key = (step.get("action"), step.get("robot_id", 1),
                   step.get("distance", step.get("angle")))
            groups.setdefault(key, []).append(i)
        for (action, rid, value), idx in groups.items():
            shown = ", ".join(map(str, idx[:10])) + (" ..." if len(idx) > 10 else "")
            logger.info(f"unverified until run: Robot {rid} {action} {value} "
                        f"（{len(idx)} 回, ステップ {shown}）")


#: プリフライトで可動域違反が見つかったときの終了コード（run_flow / run_csv 共通）
PREFLIGHT_EXIT_CODE = 3


class PreflightError(RuntimeError):
    """プリフライトで可動域違反が見つかった（デバイスは一切開いていない）。

    GUI の FlowRunner などライブラリとして呼ぶ経路はこの例外で中止する。
    CLI は同じ条件で終了コード ``PREFLIGHT_EXIT_CODE``（3）を返す。
    """

    def __init__(self, report: "PreflightReport", message: str):
        super().__init__(message)
        self.report = report


def _unverified_note(report: PreflightReport) -> str:
    if not report.unverified:
        return ""
    return (f"。相対移動 {len(report.unverified)} 件（move_z / rotate_relative / "
            "move_radial）は開始姿勢に依存するため未検証で、--mock / 実行時に "
            "1 手ごとに検査されます")


def violation_message(report: PreflightReport) -> str:
    """違反を 1 件 1 行にまとめた、UI / 例外向けの短い文面"""
    lines = []
    if report.capability_errors:
        lines.append(f"ロボットが対応していないアクションが {len(report.capability_errors)} 件あります。"
                     "デバイスを開かずに中止します:")
        lines.extend(f"- {msg}" for msg in report.capability_errors)
    if not report.violations:
        return "\n".join(lines)
    lines.append(f"可動域チェックで {len(report.violations)} 件の違反があります。"
                 "デバイスを開かずに中止します:")
    for i, step, err in report.violations[:20]:
        lines.append(f"- {_step_label(i, step)} {_fmt_params(step)}: {err}")
    if len(report.violations) > 20:
        lines.append(f"- ...ほか {len(report.violations) - 20} 件")
    return "\n".join(lines)


def robot_types_from_ports(robot_ports: dict) -> Dict[int, str]:
    """robot_id → 種別（``type`` が無ければ "dobot"）"""
    return {int(rid): robot_type(ports) for rid, ports in (robot_ports or {}).items()}


def build_robot_validators(robot_ids, validator, robot_ports: dict) -> Dict[int, object]:
    """robot_id → 可動域バリデータ（専用の workspace が無いロボットは ``validator``）"""
    return {rid: validator_for_robot(rid, validator, robot_ports) for rid in robot_ids}


def run_preflight(steps: list, validator=None, log=None, *, robot_ports: Optional[dict] = None,
                  validators_by_robot: Optional[dict] = None,
                  robot_types: Optional[dict] = None) -> PreflightReport:
    """デバイスを開く前のプリフライト（可動域 + 対応アクションの検査とログ出力）。

    run_flow.py・CSV ランナー（run_csv.py）・GUI（src/gui/runner.py）が共通で
    使う入口。呼び出し側は ``report.ok`` が False なら何も開かずに中止する
    （CLI は終了コード 3、ライブラリ経路は :class:`PreflightError`）。

    Args:
        steps: ループ展開後のステップ dict のリスト
        validator: 共通の可動域バリデータ（省略時は config.yaml の workspace）
        log: 出力先ロガー（省略時は run_flow のロガー）
        robot_ports: robot_id → ロボット設定（種別・ロボット別 workspace の出どころ）。
            省略時は config.yaml
        validators_by_robot: robot_id → バリデータ。省略時は robot_ports から作る
            （robots.<id>.workspace を持つロボットだけ専用、他は ``validator``）。
            渡した map に無い（または None の）使用ロボットは robot_ports から
            :func:`validator_for_robot` で補う（共通のバリデータで黙って埋めない）。
            MaxArm の項目が共通の（Dobot の）バリデータそのものなら ConfigError
        robot_types: robot_id → 種別。省略時は robot_ports から作る

    Returns:
        PreflightReport。``validators_by_robot`` と ``robot_types`` には実際に
        使った値が入る（ExperimentSession に渡して実行時も同じ可動域を使う）。

    Raises:
        lab_config.ConfigError: ロボットの可動域の設定が使えない場合、または
            渡された ``validators_by_robot`` が MaxArm に共通のバリデータを割り当てている場合
    """
    log = log or logger
    if validator is None:
        validator = default_workspace_validator()
    robot_ids = _robot_ids_in(steps)
    if validators_by_robot is None:
        missing = list(robot_ids)
    else:
        missing = [rid for rid in robot_ids if validators_by_robot.get(rid) is None]
    if robot_ports is None and (missing or robot_types is None):
        robot_ports = lab_config.get_robot_ports()
    # 渡された map に無いロボットも robot_ports から作る（MaxArm が共通の Dobot の
    # 可動域で検査・実行されることのないように）
    validators_by_robot = dict(validators_by_robot or {})
    validators_by_robot.update(build_robot_validators(missing, validator, robot_ports))
    if robot_types is None:
        robot_types = robot_types_from_ports(robot_ports)
    for rid in robot_ids:
        if (robot_types.get(rid) == "maxarm" and validators_by_robot[rid] is validator):
            raise lab_config.ConfigError(
                f"Robot {rid} is a MaxArm, but the supplied validators_by_robot gives it the "
                f"shared (Dobot) workspace validator. A MaxArm is checked only against its own "
                f"robots.{rid}.workspace."
            )
    # 共通の可動域と異なるロボットだけ渡す（共通のみなら従来どおりの呼び出し）
    own = {rid: v for rid, v in validators_by_robot.items() if v is not validator}
    report = (preflight_workspace(steps, validator, validators_by_robot=own) if own
              else preflight_workspace(steps, validator))
    report.validators_by_robot = {rid: validators_by_robot[rid] for rid in robot_ids}
    report.robot_types = {rid: robot_types.get(rid, lab_config.DEFAULT_ROBOT_TYPE)
                          for rid in robot_ids}
    report.capability_errors = preflight_capabilities(steps, robot_types)
    log_preflight(report, validator, log=log)
    if not report.ok:
        if report.capability_errors:
            log.error(f"ロボットが対応していないアクションが {len(report.capability_errors)} 件"
                      "あります。デバイスを開かずに中止します（上記）")
        if report.violations:
            log.error(f"可動域チェックで {len(report.violations)} 件の違反があります。"
                      "デバイスを開かずに中止します（ステップと座標は上記）")
        return report
    checked_note = (f"絶対目標 {report.checked} 件すべて範囲内" if report.checked
                    else "絶対目標（move_xyz / rotate）なし")
    log.info(f"可動域チェック: {checked_note}{_unverified_note(report)}")
    return report


def require_preflight(steps: list, validator=None, log=None, **kwargs) -> PreflightReport:
    """:func:`run_preflight` を行い、違反があれば :class:`PreflightError` を送出する

    ``kwargs`` は run_preflight の robot_ports / validators_by_robot / robot_types。
    """
    report = run_preflight(steps, validator, log=log, **kwargs)
    if not report.ok:
        raise PreflightError(report, violation_message(report))
    return report


class ResourcePlan(NamedTuple):
    """フローが開くロボット・共有デバイス（:func:`plan_resources` の戻り値）"""
    robot_ids: list
    picus2_robots: set
    needs_scale: bool
    needs_camera: bool
    needs_microscope: bool
    needs_microscope_serial: bool
    needs_dmm: bool
    needs_lockin: bool

    @property
    def needs_shared(self) -> bool:
        return (self.needs_scale or self.needs_camera or self.needs_microscope
                or self.needs_microscope_serial or self.needs_dmm or self.needs_lockin)

    def shared_kwargs(self) -> dict:
        """ExperimentSession.add_shared() に渡す use_* 引数"""
        return dict(use_scale=self.needs_scale, use_camera=self.needs_camera,
                    use_microscope=self.needs_microscope,
                    use_microscope_serial=self.needs_microscope_serial,
                    use_dmm=self.needs_dmm, use_lockin=self.needs_lockin)

    def shared_labels(self) -> list:
        """ログ表示用の共有デバイス名"""
        return [n for n, need in (("天秤", self.needs_scale), ("カメラ", self.needs_camera),
                                  ("顕微鏡カメラ", self.needs_microscope),
                                  ("顕微鏡制御ポート", self.needs_microscope_serial),
                                  ("DMM", self.needs_dmm), ("ロックイン", self.needs_lockin))
                if need]


def plan_resources(steps: list) -> ResourcePlan:
    """フローが必要とするロボット・共有デバイスを洗い出す。

    フローに出てこないデバイスは開かない。呼び出し側は名前（``plan.needs_dmm`` など）で
    読む（項目が増えても呼び出し側を壊さない）。
    """
    actions = {s.get("action") for s in steps}
    robot_ids = set()
    picus2_robots = set()
    for s in steps:
        action = s.get("action")
        if action in SHARED_DEVICE_ACTIONS or action in LOOP_ACTIONS:
            continue
        rid = s.get("robot_id", 1)
        robot_ids.add(rid)
        if action in PICUS2_ACTIONS:
            picus2_robots.add(rid)
    needs_scale = bool(actions & {"measure_weight", "tare_scale"})
    needs_camera = "capture_and_save" in actions
    needs_microscope = bool(actions & MICROSCOPE_CAMERA_ACTIONS)          # 顕微鏡カメラ
    needs_microscope_serial = bool(actions & MICROSCOPE_SERIAL_ACTIONS)   # 顕微鏡の制御ポート
    needs_dmm = bool(actions & DMM_ACTIONS)                               # Keithley 2002
    needs_lockin = bool(actions & LOCKIN_ACTIONS)                         # NF LI5640
    return ResourcePlan(sorted(robot_ids), picus2_robots, needs_scale, needs_camera,
                        needs_microscope, needs_microscope_serial, needs_dmm, needs_lockin)


def _fmt_params(step: dict) -> str:
    return " ".join(f"{k}={v}" for k, v in step.items() if k not in ("action", "_iteration"))


async def run(args) -> int:
    workflow, steps = load_and_validate(args.flow)

    # プリフライト: どのデバイスも開く前に、絶対目標の可動域（ロボットごと）と
    # 各アクションがそのロボットの種別で実行できるかを検査する
    robot_ports, shared_config = resolve_ports(args)
    validator = default_workspace_validator()
    report = run_preflight(steps, validator, robot_ports=robot_ports)
    if not report.ok:
        return PREFLIGHT_EXIT_CODE
    unverified_note = _unverified_note(report)

    if args.validate_only:
        logger.info(f"検証のみ完了（--validate-only）: スキーマ・ループ構造・絶対目標の可動域 OK"
                    f"{unverified_note}")
        return 0
    if not steps:
        logger.error("ステップが空です")
        return 2

    plan = plan_resources(steps)
    robot_ids, picus2_robots = plan.robot_ids, plan.picus2_robots
    robot_types = report.robot_types

    # 実機実行では既定で録画する。Mock モードでは常に録画しない。
    record = (not args.mock) and (True if args.record is None else args.record)

    # 実行ごとのログフォルダ（run.log / measurements.csv / summary.md /
    # metadata.json / images/ と入力 JSON のコピー）
    exp_logger = ExperimentLogger(
        workflow.name, workflow.description, args.flow, base_dir=LOGS_DIR,
        mode="mock" if args.mock else "real",
    )
    exp_logger.record_resources(robot_ids, picus2_robots, robot_types=robot_types)
    logger.info(f"実験ログ保存先: {exp_logger.dir}")

    accuracy_logger = DispenseAccuracyLogger(
        os.path.join(exp_logger.dir, "dispense_accuracy.csv"), density=args.liquid_density
    )
    logger.info(f"分注精度CSV: {accuracy_logger.csv_path} (密度={args.liquid_density}g/mL)")

    logger.info(f"使用ロボット: {robot_ids}（種別: "
                + ", ".join(f"{rid}={robot_types.get(rid, 'dobot')}" for rid in robot_ids) + "）")
    if picus2_robots:
        logger.info(f"ピペット使用: Robot {sorted(picus2_robots)}")
    if plan.needs_shared:
        logger.info(f"共有デバイス: {', '.join(plan.shared_labels())}")

    # 録画セッション開始（CSV: ダッシュボード経由 / 動画: ローカルスレッド）
    video_recorder = None
    recording = None
    if record:
        if ensure_dashboard_running():
            recording = start_csv_recording(workflow.name)
            exp_logger.record_recording(recording.as_metadata())
            save_dir = recording.save_dir or exp_logger.dir
            try:
                from src.monitoring.camera.video_recorder import VideoRecorder
                video_recorder = VideoRecorder(
                    output_path=os.path.join(
                        save_dir, f"{os.path.basename(exp_logger.dir)}.mp4"),
                    camera_index=lab_config.get_video_camera_index(),
                )
                if not video_recorder.start():
                    logger.warning("動画録画の開始に失敗しました（CSV録画は継続します）")
                    video_recorder = None
            except ImportError as e:
                logger.warning(f"動画録画を利用できません（OpenCV 未導入?）: {e}")
                video_recorder = None
        else:
            logger.warning("ダッシュボード未起動のため録画をスキップします")

    session = ExperimentSession(
        mock=args.mock,
        robot_ports=robot_ports,
        shared_config=shared_config,
        workspace_validator=validator,
        robot_validators=report.validators_by_robot,
    )

    async def body():
        for rid in robot_ids:
            await session.add_robot(rid, use_picus2=(rid in picus2_robots))
        if plan.needs_shared:
            await session.add_shared(**plan.shared_kwargs())

        logger.info("=== フロー実行開始 ===")
        total = len(steps)
        for i, step in enumerate(steps, 1):
            action = step.get("action")
            rid = None
            if action in SHARED_DEVICE_ACTIONS:
                logger.info(f"[{i}/{total}] 共有: {action} {_fmt_params(step)}")
            else:
                rid = step.get("robot_id", 1)
                logger.info(f"[{i}/{total}] Robot {rid}: {action} {_fmt_params(step)}")

            # 撮影画像は実験フォルダ内に束ねる（file_path 未指定の場合のみ）
            if action == "capture_and_save" and not step.get("file_path"):
                step = {**step, "file_path": exp_logger.next_image_path(i)}
            elif action == "capture_microscope" and not step.get("file_path"):
                step = {**step, "file_path": exp_logger.next_image_path(i, tag="microscope")}

            iteration = step.get("_iteration")
            t_start = time.monotonic()
            try:
                result = await execute_step(step, session.robots, session.shared, logger)
                exp_logger.record_step(i, total, action, rid, iteration,
                                       "ok", time.monotonic() - t_start, result=result)
            except Exception as step_err:
                exp_logger.record_step(i, total, action, rid, iteration,
                                       "error", time.monotonic() - t_start,
                                       error=str(step_err),
                                       diagnostics=getattr(step_err, "diagnostics", None))
                raise

            # 分注精度: dispense の指示量と直後の measure_weight を対応付ける
            if action == "dispense":
                accuracy_logger.record_dispense(
                    volume=step.get("volume"), speed=step.get("speed"),
                    iteration=iteration, step_index=i,
                )
            elif action == "measure_weight":
                weight = result.get("weight") if isinstance(result, dict) else result
                err = accuracy_logger.record_weight(weight)
                if err is not None:
                    logger.info(f"  分注誤差: {err:+.4f}g")

        logger.info("=== フロー実行完了 ===")

    exit_code = 0
    try:
        await session.run(body)
    except (KeyboardInterrupt, asyncio.CancelledError):
        exit_code = 130
    except Exception as e:  # noqa: BLE001 - 実行時エラーは終了コードで伝える
        logger.error(f"フロー実行に失敗しました: {e}")
        exit_code = 1
    finally:
        if video_recorder is not None:
            try:
                video_recorder.stop()
            except Exception as e:  # noqa: BLE001
                logger.warning(f"動画録画の停止でエラー: {e}")
        if recording is not None:
            try:
                stop_csv_recording(recording)
            except Exception as e:  # noqa: BLE001
                logger.warning(f"CSV録画の停止でエラー: {e}")
        try:
            accuracy_logger.finalize(logger)
        except Exception as e:  # noqa: BLE001
            logger.warning(f"分注精度CSVの確定でエラー: {e}")
        exp_logger.finalize(session.status, session.error)
        logger.info(f"実験ログを保存しました: {exp_logger.dir}")

    return exit_code


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    if not os.path.exists(args.flow):
        logger.error(f"ファイルが見つかりません: {args.flow}")
        return 2
    try:
        return asyncio.run(run(args))
    except ValidationError as e:
        logger.error(f"JSON スキーマ検証に失敗しました:\n{e}")
        return 2
    except json.JSONDecodeError as e:
        logger.error(f"JSON を読み込めません: {e}")
        return 2
    except ValueError as e:
        # ループ構造の誤り（expand_loops）など
        logger.error(f"フローが不正です: {e}")
        return 2
    except lab_config.ConfigError as e:
        logger.error(f"設定ファイルを使えません（組み込み既定値には戻しません）: {e}")
        return 2
    except KeyboardInterrupt:
        # 緊急停止とクリーンアップは run() 内の ExperimentSession が実施済み
        logger.warning("中断されました（緊急停止・クリーンアップ実施済み）")
        return 130


if __name__ == "__main__":
    sys.exit(main())
