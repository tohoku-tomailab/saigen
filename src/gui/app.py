import streamlit as st
import json
import logging
import sys
import os
import time
from html import escape

import glob as glob_module

import requests

# ==========================================
# 1. パス設定 & モジュールインポート
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(current_dir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from src.agent.llm_service import generate_workflow_from_prompt, LLM_AVAILABLE
except ImportError:
    LLM_AVAILABLE = False
    def generate_workflow_from_prompt(x, template=None): return {}

try:
    from src.voice.speech_service import transcribe_audio, WHISPER_AVAILABLE
    from streamlit_mic_recorder import mic_recorder
    VOICE_INPUT_AVAILABLE = True
except ImportError:
    VOICE_INPUT_AVAILABLE = False

from pydantic import ValidationError

from src.devices.dso1000 import ITEMS as SCOPE_ITEMS
from src.devices.keithley2002 import FUNCTIONS as DMM_FUNCTIONS
from src.flow.executor import SHARED_DEVICE_ACTIONS
from src.flow.schema import PIPETTE_MIN_VOLUME_ML

# 実行ロジックは Streamlit 非依存の runner.py に集約している（CLI と同じ経路:
# スキーマ検証 → ループ展開 → ExperimentSession → ExperimentLogger）。
from src.gui import runner as flow_runner

SENSOR_SERVER_URL = os.getenv("SENSOR_SERVER_URL", "http://localhost:8000")


def _load_presets():
    """presets/ディレクトリからプリセットを読み込む"""
    presets = {}
    preset_dir = os.path.join(REPO_ROOT, "src", "agent", "presets")
    if not os.path.isdir(preset_dir):
        return presets
    for path in sorted(glob_module.glob(os.path.join(preset_dir, "*.json"))):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("hidden"):
            continue
        key = os.path.splitext(os.path.basename(path))[0]
        presets[key] = data
    return presets


PRESET_TEMPLATES = _load_presets()

# ==========================================
# 2. 定数・マッピング定義（日本語化）
# ==========================================
ACTION_CONFIG = {
    "move_xyz": {
        "icon": "🚚", "label": "座標移動", "category": "robot",
        "defaults": {"x": 200.0, "y": 0.0, "z": 50.0}
    },
    "move_z": {
        "icon": "⬆️", "label": "Z軸移動", "category": "robot",
        "defaults": {"distance": 10.0}
    },
    "move_radial": {
        "icon": "↔️", "label": "半径移動", "category": "robot",
        "defaults": {"distance": 20.0}
    },
    "rotate": {
        "icon": "🔄", "label": "絶対回転", "category": "robot",
        "defaults": {"angle": 90.0}
    },
    "rotate_relative": {
        "icon": "🔄", "label": "回転", "category": "robot",
        "defaults": {"angle": 90.0, "speed": "low"}
    },
    "go_home": {
        "icon": "🏠", "label": "ホーム", "category": "robot",
        "defaults": {}
    },
    "move_slider": {
        "icon": "🛤️", "label": "スライダー", "category": "peripheral",
        "defaults": {"position": 500.0}
    },
    "move_conveyer": {
        "icon": "🔃", "label": "コンベア", "category": "peripheral",
        "defaults": {"index": 0, "speed": 10.0, "duration": 5.0}
    },
    "wait": {
        "icon": "⏳", "label": "待機", "category": "utility",
        "defaults": {"seconds": 2.0}
    },
    "aspirate": {
        "icon": "💧", "label": "吸引", "category": "pipette",
        "defaults": {"volume": 5.0, "speed": 5}
    },
    "dispense": {
        "icon": "💦", "label": "吐出", "category": "pipette",
        "defaults": {"volume": 5.0, "speed": 5}
    },
    "blow_out": {
        "icon": "💨", "label": "排出", "category": "pipette",
        "defaults": {"go_home": True, "speed": 1, "delay_ms": 3000}
    },
    # MaxArm（吸着ノズル）。追加時の robot_id は MaxArm のロボット（add_step）
    "suction": {
        "icon": "🧲", "label": "吸着 ON/OFF", "category": "maxarm",
        "defaults": {"on": True}
    },
    "capture_and_save": {
        "icon": "📷", "label": "撮影", "category": "sensor",
        "defaults": {"file_path": ""}
    },
    "capture_microscope": {
        "icon": "🔬", "label": "顕微鏡撮影", "category": "sensor",
        "defaults": {"file_path": ""}
    },
    "microscope_led": {
        "icon": "💡", "label": "顕微鏡LED", "category": "sensor",
        "defaults": {"on": True, "level": None}
    },
    "microscope_focus": {
        "icon": "🎯", "label": "顕微鏡フォーカス", "category": "sensor",
        "defaults": {"mode": "auto", "position": None, "direction": "in", "steps": 1, "timeout": 60.0}
    },
    "measure_weight": {
        "icon": "⚖️", "label": "計測", "category": "sensor",
        "defaults": {"stabilization_count": 3}
    },
    "tare_scale": {
        "icon": "🔄", "label": "風袋引き", "category": "sensor",
        "defaults": {"delay": 1.0}
    },
    "measure_dmm": {
        "icon": "🔌", "label": "DMM測定", "category": "sensor",
        "defaults": {"function": "dc_voltage", "samples": 3, "nplc": 1.0}
    },
    "measure_lockin": {
        "icon": "📈", "label": "ロックイン測定", "category": "sensor",
        "defaults": {"samples": 5, "interval": 0.0}
    },
    "set_lockin_oscillator": {
        "icon": "🔊", "label": "ロックイン発振器設定", "category": "sensor",
        "defaults": {"frequency": 100.0, "amplitude": 0.1, "settle": 1.0}
    },
    "measure_scope": {
        "icon": "📉", "label": "オシロ測定", "category": "sensor",
        "defaults": {"channel": 1, "items": ["vpp", "frequency"], "samples": 3, "interval": 0.5}
    },
    # ループ制御
    "loop_start": {
        "icon": "🔁", "label": "ループ開始", "category": "loop",
        "defaults": {"loop_id": "", "count": 10}
    },
    "loop_end": {
        "icon": "🔚", "label": "ループ終了", "category": "loop",
        "defaults": {"loop_id": ""}
    },
}

# Robot色定義
ROBOT_COLORS = {
    1: {"bg": "#e3f2fd", "border": "#1976d2", "label": "🔵", "name": "Robot 1"},
    2: {"bg": "#e8f5e9", "border": "#388e3c", "label": "🟢", "name": "Robot 2"},
    3: {"bg": "#fff3e0", "border": "#e65100", "label": "🟠", "name": "Robot 3 (Slider)"},
    4: {"bg": "#f3e5f5", "border": "#7b1fa2", "label": "🟣", "name": "Robot 4 (MaxArm)"},
}

SHARED_COLOR = {"bg": "#f5f5f5", "border": "#9e9e9e"}

# カテゴリ定義（日本語）
CATEGORIES = {
    "robot": ("🤖 ロボット", ["move_xyz", "move_z", "move_radial", "rotate_relative", "go_home"]),
    "peripheral": ("🔩 周辺機器", ["move_slider", "move_conveyer"]),
    "pipette": ("💉 ピペット", ["aspirate", "dispense", "blow_out"]),
    "maxarm": ("🧲 MaxArm", ["suction"]),
    "sensor": ("📡 センサー", ["capture_and_save", "capture_microscope", "microscope_led", "microscope_focus", "measure_weight", "tare_scale", "measure_dmm", "measure_lockin", "set_lockin_oscillator", "measure_scope"]),
    "utility": ("⚙️ その他", ["wait"]),
    "loop": ("🔄 ループ", ["loop_start"]),  # loop_endはloop_start追加時に自動追加
}

# ループ色定義
LOOP_COLORS = {
    "bg": "#fff3e0",
    "border": "#ff9800",
    "bracket": "#e65100"
}

# ==========================================
# 3. ページ設定
# ==========================================
st.set_page_config(
    page_title="Visual Automation Controller",
    page_icon="🧩",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==========================================
# 4. 状態管理 (Session State)
# ==========================================
DEFAULT_WORKFLOW = {
    "name": "New Experiment",
    "description": "Created with Visual Editor",
    "steps": []
}

if "workflow_data" not in st.session_state:
    st.session_state.workflow_data = DEFAULT_WORKFLOW

# 接続ポートの既定値は config.yaml（無ければ config.example.yaml）から取る。
# サイドバーで編集した値はそのセッション内だけ有効（恒久的な変更は config.yaml へ）。
if "robot_configs" not in st.session_state or "shared_config" not in st.session_state:
    _device_defaults = flow_runner.default_device_settings()
    st.session_state.robot_configs = _device_defaults["robots"]
    st.session_state.shared_config = _device_defaults["shared"]
else:
    # 以前のバージョンで作られたセッション: 追加されたロボット（Robot 4 など）を補う。
    # config.yaml でロボットの種別が変わった（Dobot ⇔ MaxArm）エントリは、古い種別の
    # ポートを持ち越さないよう config.yaml の値で置き換え、そのウィジェットの状態も捨てる
    # （捨てないと古い入力値がウィジェットに残り、置き換えた値を上書きする）
    for _rid, _cfg in flow_runner.default_device_settings()["robots"].items():
        _old = st.session_state.robot_configs.get(_rid)
        if _old is None:
            st.session_state.robot_configs[_rid] = _cfg
        elif _old.get("type") != _cfg.get("type"):
            st.session_state.robot_configs[_rid] = _cfg
            for _suffix in ("_dp", "_picus2", "_pp", "_mp"):
                st.session_state.pop(f"r{_rid}{_suffix}", None)

# 実行モード: 既定は Mock。実機で動かすには実行パネルの「実機で実行する」を
# このセッション内で毎回チェックし、さらに確認ダイアログで承認する必要がある。
# 実機ランを開始するとチェックは自動で外れる（次の実機ランでは再度チェックが要る）。
if "arm_real_run" not in st.session_state:
    st.session_state.arm_real_run = False
if "flow_rev" not in st.session_state:
    st.session_state.flow_rev = 0        # ウィジェットキーに含めるフローの版番号
if "rejected_flow" not in st.session_state:
    st.session_state.rejected_flow = None  # 検証に通らず読み込まなかったフロー（読み取り専用表示）


def generate_loop_id():
    """一意なループIDを生成"""
    import time
    return f"loop_{int(time.time() * 1000) % 100000}"


def add_step(action_type):
    """新しいブロックを追加"""
    if is_running():
        return
    _bump_flow_rev()
    config = ACTION_CONFIG.get(action_type, {})
    new_step = {"action": action_type, **config.get("defaults", {})}

    # ループ開始: loop_endも同時に追加
    if action_type == "loop_start":
        loop_id = generate_loop_id()
        new_step["loop_id"] = loop_id
        st.session_state.workflow_data["steps"].append(new_step)
        st.session_state.workflow_data["steps"].append({
            "action": "loop_end",
            "loop_id": loop_id
        })
        return

    # ループ終了: 単独追加は禁止
    if action_type == "loop_end":
        st.warning("ループ終了は「ループ開始」追加時に自動で追加されます")
        return

    if action_type not in SHARED_DEVICE_ACTIONS:
        new_step["robot_id"] = _default_robot_for(action_type)
    st.session_state.workflow_data["steps"].append(new_step)


def _default_robot_for(action_type):
    """新しいブロックの robot_id。吸着は最初の MaxArm、それ以外は Robot 1"""
    if action_type == "suction":
        for rid, cfg in sorted(st.session_state.robot_configs.items()):
            if cfg.get("type") == "maxarm":
                return rid
    return 1


def remove_step(index):
    """ステップを削除（ループの場合はペアで削除）"""
    if is_running():
        return
    _bump_flow_rev()
    steps = st.session_state.workflow_data["steps"]
    step = steps[index]
    action = step.get("action")

    # ループの場合はペアで削除
    if action in ["loop_start", "loop_end"]:
        loop_id = step.get("loop_id")
        # 同じloop_idを持つステップのインデックスを収集
        indices_to_remove = []
        for i, s in enumerate(steps):
            if s.get("loop_id") == loop_id and s.get("action") in ["loop_start", "loop_end"]:
                indices_to_remove.append(i)
        # 後ろから削除（インデックスずれ防止）
        for i in sorted(indices_to_remove, reverse=True):
            steps.pop(i)
    else:
        steps.pop(index)


def move_step(index, direction):
    if is_running():
        return
    _bump_flow_rev()
    steps = st.session_state.workflow_data["steps"]
    new_index = index + direction
    if 0 <= new_index < len(steps):
        steps[index], steps[new_index] = steps[new_index], steps[index]


def _update_robot_id(idx, key):
    """ラジオボタンのon_changeコールバック用"""
    selected = st.session_state[key]
    # ラベルからrobot_idを逆引き
    for rid, color in ROBOT_COLORS.items():
        if selected == f"{color['label']} {color['name']}":
            st.session_state.workflow_data["steps"][idx]["robot_id"] = rid
            return
    st.session_state.workflow_data["steps"][idx]["robot_id"] = 1


def _update_step_param(idx, param_name, widget_key):
    """ウィジェットの値変更時にstepデータを即時更新するコールバック

    フローへの書き込みはこのコールバック（= ユーザーが値を変えたとき）だけで行う。
    実行中は書き換えない。
    """
    if is_running():
        return
    st.session_state.workflow_data["steps"][idx][param_name] = st.session_state[widget_key]


# ==========================================
# 5. 実行状態 (Session State)
# ==========================================
if "runner" not in st.session_state:
    st.session_state.runner = None       # 実行中/直近の FlowRunner
if "log_messages" not in st.session_state:
    st.session_state.log_messages = []
if "error_message" not in st.session_state:
    st.session_state.error_message = None
if "run_status" not in st.session_state:
    st.session_state.run_status = None   # completed / failed / aborted
if "run_log_dir" not in st.session_state:
    st.session_state.run_log_dir = None
if "run_progress" not in st.session_state:
    st.session_state.run_progress = (0, 0)  # (完了ステップ数, 総ステップ数)
if "experiment_recording" not in st.session_state:
    st.session_state.experiment_recording = False


def add_log(message: str):
    """ログメッセージをセッションに追加"""
    from datetime import datetime
    timestamp = datetime.now().strftime('%H:%M:%S')
    st.session_state.log_messages.append(f"{timestamp} - {message}")


def is_running() -> bool:
    runner = st.session_state.runner
    return runner is not None and runner.is_running()


# ==========================================
# 6. ワークフロー実行（runner.py に委譲）
# ==========================================
def start_experiment_session(name: str, description: str = None) -> dict:
    """センサーサーバーに実験開始を通知 (sensor_web_dashboard /api/start)

    200 のときだけ「このランが開始した録画」として記録し、終了時に /api/end を送る。
    409（既に録画中）は他の誰かの録画なので、止めない。

    Returns:
        metadata.json に残す録画情報（``started_by_this_run`` など）
    """
    st.session_state.experiment_recording = False
    info = {"started_by_this_run": False, "save_dir": None, "start_status": 0}
    try:
        resp = requests.post(
            f"{SENSOR_SERVER_URL}/api/start",
            json={"experiment_name": name},
            timeout=3,
        )
        info["start_status"] = resp.status_code
        if resp.status_code == 200:
            data = resp.json()
            st.session_state.experiment_recording = True
            info.update(started_by_this_run=True, save_dir=data.get("save_dir"))
            add_log(f"センサー録画開始 (save_dir={data.get('save_dir')})")
        elif resp.status_code == 409:
            add_log("センサー録画開始スキップ: 既に録画中（このランの録画ではないので終了時も止めません）")
        else:
            add_log(f"センサー録画開始失敗: {resp.status_code} {resp.text}")
    except requests.RequestException:
        pass
    return info


def stop_experiment_session():
    """センサーサーバーに実験終了を通知 (sensor_web_dashboard /api/end)"""
    if not st.session_state.get("experiment_recording"):
        return
    try:
        resp = requests.post(
            f"{SENSOR_SERVER_URL}/api/end",
            timeout=3,
        )
        if resp.status_code == 200:
            saved_path = resp.json().get("saved_path")
            add_log(f"センサー録画終了 (saved_path={saved_path})")
        elif resp.status_code == 409:
            add_log("センサー録画終了スキップ: 録画していません")
        else:
            add_log(f"センサー録画終了失敗: {resp.status_code} {resp.text}")
    except requests.RequestException:
        pass
    finally:
        st.session_state.experiment_recording = False


def build_run_settings() -> dict:
    """サイドバーの設定を runner が扱う形にまとめる"""
    return {
        "robots": st.session_state.robot_configs,
        "shared": st.session_state.shared_config,
    }


def set_workflow(data, source: str = "生成結果") -> bool:
    """LLM・プリセット・JSON読込のフローを、検証に通った場合だけキャンバスに載せる

    検証に失敗したフローは読み込まず（今のフローはそのまま）、読み取り専用で
    エラーと一緒に表示する。実行中は差し替えない。

    Returns:
        読み込んだら True
    """
    if is_running():
        st.session_state.error_message = "実行中はフローを差し替えられません（停止または完了後に読み込んでください）"
        return False
    try:
        if not isinstance(data, dict):
            raise TypeError("フローは JSON オブジェクトである必要があります")
        flow_runner.validate_workflow(data)
    except ValidationError as e:
        message = (f"{source}はスキーマ検証に失敗したため読み込みませんでした"
                   "（今のフローはそのままです）:\n\n"
                   + flow_runner.format_validation_error(e, data if isinstance(data, dict) else None))
    except (ValueError, TypeError) as e:
        message = f"{source}のフロー定義エラーのため読み込みませんでした（今のフローはそのままです）: {e}"
    else:
        st.session_state.workflow_data = data
        st.session_state.rejected_flow = None
        st.session_state.error_message = None
        _bump_flow_rev()
        return True
    st.session_state.rejected_flow = {"source": source, "data": data, "error": message}
    st.session_state.error_message = message
    return False


def validate_current_workflow():
    """実行せずにスキーマ検証・ループ展開・可動域プリフライトだけ行う"""
    try:
        _, steps = flow_runner.validate_workflow(st.session_state.workflow_data)
        report = flow_runner.require_preflight(steps)
    except ValidationError as e:
        st.session_state.error_message = (
            "JSON スキーマ検証に失敗しました:\n\n"
            + flow_runner.format_validation_error(e, st.session_state.workflow_data)
        )
    except flow_runner.PreflightError as e:
        st.session_state.error_message = f"可動域プリフライトで違反が見つかりました:\n\n{e}"
    except (ValueError, TypeError) as e:
        st.session_state.error_message = f"フロー定義エラー: {e}"
    else:
        st.session_state.error_message = None
        note = (f"、相対移動 {len(report.unverified)} 件は実行時に検査"
                if report.unverified else "")
        st.toast(f"検証OK: {len(steps)} ステップ（ループ展開後）{note}", icon="✅")


def start_run(mock: bool = True):
    """検証・プリフライトしてから FlowRunner を起動する（実行はバックグラウンドスレッド1本）

    検証または可動域プリフライトに失敗したフローはここで弾かれ、ロボットには
    一切触れない。``mock=False``（実機）は、実行パネルで「実機で実行する」を
    チェックし確認ダイアログで承認したときだけ渡される。
    """
    if is_running():
        return
    st.session_state.log_messages = []
    st.session_state.error_message = None
    st.session_state.run_status = None
    st.session_state.run_log_dir = None
    st.session_state.run_progress = (0, 0)

    use_mock = bool(mock)
    robot_ports, shared_config = flow_runner.settings_to_session_args(build_run_settings())

    try:
        runner = flow_runner.FlowRunner(
            st.session_state.workflow_data,
            mock=use_mock,
            robot_ports=robot_ports,
            shared_config=shared_config,
            # Mock 実行では録画も安全確認ゲートも使わない（CLI の --mock と同じ）
            pre_step=None if use_mock else flow_runner._safety_gate,
        )
    except ValidationError as e:
        st.session_state.error_message = (
            "JSON スキーマ検証に失敗しました。実行は行いません:\n\n"
            + flow_runner.format_validation_error(e, st.session_state.workflow_data)
        )
        return
    except flow_runner.PreflightError as e:
        st.session_state.error_message = f"可動域プリフライトで違反が見つかりました。実行は行いません:\n\n{e}"
        return
    except (ValueError, TypeError) as e:
        st.session_state.error_message = f"フロー定義エラー。実行は行いません: {e}"
        return

    add_log(f"実験開始: {runner.workflow.name}")
    add_log(f"モード: {'🛠 MOCK' if use_mock else '⚠️ REAL'}")
    add_log(f"✓ 検証OK: {len(runner.steps)} ステップ（ループ展開後）")

    if not use_mock:
        runner.recording = start_experiment_session(
            name=runner.workflow.name, description=runner.workflow.description
        )

    st.session_state.run_progress = (0, len(runner.steps))
    st.session_state.runner = runner
    runner.start()


def drain_runner_events():
    """runner が積んだ進捗イベントをログ・進捗・完了状態に反映する"""
    runner = st.session_state.runner
    if runner is None:
        return
    done, total = st.session_state.run_progress

    for event in runner.drain_events():
        if event.kind == "log":
            st.session_state.log_messages.append(event.message)

        elif event.kind == "start":
            total = event.total or total
            st.session_state.run_log_dir = event.log_dir
            add_log(f"📁 実験ログ: {event.log_dir}")

        elif event.kind == "step":
            done = event.index or done
            total = event.total or total
            if event.status == "error":
                add_log(f"❌ [{event.index}/{total}] {event.action}: {event.error}")

        elif event.kind == "finish":
            st.session_state.run_status = event.status
            st.session_state.run_log_dir = event.log_dir or st.session_state.run_log_dir
            stop_experiment_session()
            if event.status == "completed":
                add_log("✅ 全ステップ完了")
            elif event.status == "aborted":
                add_log("⛔ 停止しました（緊急停止・切断まで実施済み）")
            else:
                add_log(f"❌ 実行エラー: {event.error}")
                st.session_state.error_message = f"実行エラー: {event.error}"
            add_log(f"📁 実験ログを保存しました: {event.log_dir}")

    st.session_state.run_progress = (done, total)


@st.dialog("実機で実行しますか？")
def confirm_real_run_dialog():
    """実機ランの最終確認。ここで「実機で開始」を押したときだけ mock=False で起動する"""
    data = st.session_state.workflow_data
    st.warning("⚠️ ロボットアームが動き、液体が分注されます。"
               "可動域に人や物がないこと、緊急時は ■ 停止 で止まることを確認してください。")
    st.markdown(f"**フロー**: {escape(str(data.get('name', '')))}　／　"
                f"**ステップ数**: {len(data.get('steps') or [])}（ループ展開前）")
    ports = flow_runner.settings_to_session_args(build_run_settings())[0]
    st.caption("接続ポート: " + ", ".join(
        f"Robot {rid}: " + ((f"MaxArm {cfg.get('maxarm_port') or '-'}")
                            if cfg.get("type") == "maxarm" else (cfg.get('dobot_port') or '-'))
        for rid, cfg in ports.items()))
    c1, c2 = st.columns(2)
    if c1.button("実機で開始", type="primary", use_container_width=True, key="confirm_real_run"):
        st.session_state["_disarm_real_run"] = True
        start_run(mock=False)
        st.rerun()
    if c2.button("キャンセル", use_container_width=True, key="cancel_real_run"):
        st.rerun()


def render_execution_panel():
    """実行パネル（停止 / 検証 / 実行）。キャンバスより先に描画する

    キャンバスの描画で例外が出ても停止ボタンが消えないよう、メイン領域の
    最初に置く。
    """
    running = is_running()
    # 実機ランを開始したら「実機で実行する」を外す（ウィジェット生成前にだけ変更できる）
    if st.session_state.pop("_disarm_real_run", False):
        st.session_state.arm_real_run = False

    st.markdown('<div class="execution-panel">', unsafe_allow_html=True)

    col_stop, col_validate, col_run = st.columns([1, 1, 2])
    with col_stop:
        stop_clicked = st.button(
            "■ 停止", type="secondary", use_container_width=True,
            disabled=not running, key="emergency_stop_btn",
            help="実行中のフローを中断し、ロボットを緊急停止して切断します",
        )
    with col_validate:
        validate_clicked = st.button(
            "✔ 検証", use_container_width=True, disabled=running,
            help="実行せずにスキーマ検証・ループ展開・可動域プリフライトだけ行います",
        )
    with col_run:
        armed = bool(st.session_state.get("arm_real_run"))
        run_clicked = st.button(
            "▶ 実機で実行" if armed else "▶ 実行（Mock）",
            type="primary", use_container_width=True, disabled=running,
        )

    st.checkbox(
        "実機で実行する", key="arm_real_run", disabled=running,
        help="チェックしない限り Mock（実機に触れない）で実行します。"
             "チェックした場合も確認ダイアログで承認するまで実機は動きません。"
             "実機ランを開始するとチェックは外れます。",
    )
    if st.session_state.get("arm_real_run"):
        st.warning("⚠️ 実機モード: ▶ で確認ダイアログが開き、承認するとロボットが動きます")
    else:
        st.caption("🛠 Mock モード（既定）: 実機には触れません（録画・安全確認ゲートなし）")

    st.markdown('</div>', unsafe_allow_html=True)

    # 進捗バーと保存先
    done, total = st.session_state.run_progress
    if total:
        label = f"{done}/{total} ステップ"
        if st.session_state.run_status:
            label = f"{label}（{st.session_state.run_status}）"
        st.progress(min(done / total, 1.0), text=label)
    if st.session_state.run_log_dir:
        st.caption(f"📁 {st.session_state.run_log_dir}")

    if stop_clicked and st.session_state.runner is not None:
        st.session_state.runner.request_stop()
        add_log("🛑 停止を要求しました（緊急停止 → 後始末）")
        st.rerun()

    if validate_clicked:
        validate_current_workflow()
        st.rerun()

    if run_clicked and not running:
        if armed:
            confirm_real_run_dialog()
        else:
            start_run(mock=True)
            st.rerun()


def render_toolbox():
    """ツールボックス（カテゴリ別、日本語ボタン）"""
    st.markdown("##### 🧰 ツール")

    for cat_key, (cat_label, actions) in CATEGORIES.items():
        with st.expander(cat_label, expanded=False):
            for action in actions:
                config = ACTION_CONFIG[action]
                btn_label = f"{config['icon']} {config['label']}"
                if st.button(btn_label, key=f"add_{action}", use_container_width=True,
                             disabled=is_running()):
                    add_step(action)
                    st.rerun()


def _apply_voice_result(key_suffix: str, text_area_key: str):
    """text_area描画前に呼び出し、音声入力の結果があればsession_stateに転記"""
    voice_key = f"_voice_result_{key_suffix}"
    if voice_key in st.session_state:
        st.session_state[text_area_key] = st.session_state.pop(voice_key)


def _apply_voice_result_append(key_suffix: str, text_area_key: str):
    """音声入力の結果を既存テキストに追記（プリセット用）"""
    voice_key = f"_voice_result_{key_suffix}"
    if voice_key in st.session_state:
        voice_text = st.session_state.pop(voice_key)
        existing = st.session_state.get(text_area_key, "")
        if existing.strip():
            st.session_state[text_area_key] = existing.rstrip() + "\n\n変更指示: " + voice_text
        else:
            st.session_state[text_area_key] = voice_text


def render_voice_input(key_suffix: str):
    """音声入力ボタンを描画し、文字起こし結果を一時キーに保存"""
    if not VOICE_INPUT_AVAILABLE:
        return
    audio = mic_recorder(
        start_prompt="🎤",
        stop_prompt="⏹",
        just_once=True,
        use_container_width=True,
        key=f"recorder_{key_suffix}"
    )
    if audio is not None:
        with st.spinner("文字起こし中..."):
            try:
                text = transcribe_audio(audio["bytes"])
                if text:
                    st.session_state[f"_voice_result_{key_suffix}"] = text
                    st.session_state["_reopen_dialog"] = (key_suffix == "dialog")
                    st.rerun()
            except Exception as e:
                st.error(f"文字起こしエラー: {e}")


@st.dialog("AIでワークフロー生成")
def ai_workflow_dialog():
    """AIによるワークフロー生成ダイアログ"""
    if not LLM_AVAILABLE:
        st.warning("AI生成を使うには .env に GEMINI_API_KEY（または LLM_MODE=openai_compatible と LLM_BASE_URL）を設定してください")
        return

    _apply_voice_result("dialog", "dialog_prompt")

    ai_prompt = st.text_area(
        "やりたいことを入力してください",
        placeholder="例: 90度回転して2秒待つ",
        height=150,
        key="dialog_prompt"
    )

    running = is_running()
    btn_col, mic_col = st.columns([5, 1], vertical_alignment="bottom")
    generate_clicked = btn_col.button("生成実行", type="primary", use_container_width=True,
                                      disabled=running)
    with mic_col:
        render_voice_input("dialog")
    if running:
        st.info("実行中は新しいフローを生成・読み込みできません")

    if generate_clicked and not running:
        if ai_prompt.strip():
            with st.spinner("生成中..."):
                try:
                    result = generate_workflow_from_prompt(ai_prompt)
                except Exception as e:
                    # 失敗時は何も読み込まず、入力した指示はそのまま残す
                    st.error(f"生成に失敗しました（フローは変更していません）: {e}")
                else:
                    if set_workflow(result, "AI生成"):
                        st.rerun()
                    else:
                        st.error(st.session_state.error_message)
        else:
            st.warning("指示を入力してください")


def get_param_summary(step):
    """ステップのパラメータサマリーを取得"""
    action = step["action"]

    if action == "move_xyz":
        return f"X: {step.get('x', 0)} Y: {step.get('y', 0)} Z: {step.get('z', 0)}"
    elif action == "move_z":
        return f"距離: {step.get('distance', 0)}mm"
    elif action == "move_radial":
        d = step.get('distance', 0)
        direction = "外向き" if d >= 0 else "内向き"
        return f"{abs(d)}mm {direction}"
    elif action in ["rotate", "rotate_relative"]:
        speed_labels = {"low": "低速", "normal": "普通", "high": "高速"}
        speed = speed_labels.get(step.get("speed", ""), "")
        speed_str = f"　｜　回転速度: {speed}" if speed else ""
        return f"角度: {step.get('angle', 0)}°{speed_str}"
    elif action == "move_slider":
        return f"位置: {step.get('position', 0)}mm"
    elif action == "move_conveyer":
        return f"速度: {step.get('speed', 0)}mm/s {step.get('duration', 0)}秒"
    elif action == "wait":
        return f"{step.get('seconds', 0)}秒"
    elif action == "aspirate":
        return f"{step.get('volume', 0)}mL 速度: {step.get('speed', 5)}"
    elif action == "dispense":
        return f"{step.get('volume', 0)}mL 速度: {step.get('speed', 5)}"
    elif action == "blow_out":
        return f"速度: {step.get('speed', 1)} 遅延: {step.get('delay_ms', 0)}ms"
    elif action == "suction":
        return "吸着 ON" if step.get("on", True) else "吸着 OFF（解放）"
    elif action == "capture_and_save":
        path = step.get('file_path', '')
        return escape(path) if path else "(自動)"
    elif action == "capture_microscope":
        path = step.get('file_path', '')
        return escape(path) if path else "(自動)"
    elif action == "microscope_led":
        lv = step.get('level')
        return ("点灯" if step.get('on', True) else "消灯") + (f" 明るさ: {lv}" if lv is not None else "")
    elif action == "microscope_focus":
        mode = step.get('mode', 'auto')
        if mode == "position":
            return f"位置指定: {step.get('position')}"
        if mode == "step":
            return f"ステップ {step.get('direction', 'in')} x{step.get('steps', 1)}"
        return f"AF (最大 {step.get('timeout', 60.0)}秒)"
    elif action == "measure_weight":
        return f"測定: {step.get('stabilization_count', 3)}回"
    elif action == "tare_scale":
        return f"待機: {step.get('delay', 1.0)}秒"
    elif action == "measure_dmm":
        rng = step.get('range')
        return (f"{escape(str(step.get('function', 'dc_voltage')))} {step.get('samples', 3)}回 "
                f"NPLC {step.get('nplc', 1.0)} " + (f"レンジ {rng}" if rng is not None else "オート"))
    elif action == "measure_lockin":
        return f"{step.get('samples', 5)}回 間隔 {step.get('interval', 0.0)}秒"
    elif action == "set_lockin_oscillator":
        return (f"{step.get('frequency', 100.0)} Hz {step.get('amplitude', 0.1)} Vrms "
                f"待機 {step.get('settle', 0.0)}秒")
    elif action == "measure_scope":
        items = ", ".join(escape(str(i)) for i in (step.get("items") or ["vpp", "frequency"]))
        return f"CH{step.get('channel', 1)} {items} {step.get('samples', 3)}回"
    elif action == "go_home":
        return ""
    elif action == "loop_start":
        return f"x{step.get('count', 10)}回"
    elif action == "loop_end":
        return ""
    return ""


def _wkey(name, idx):
    """ウィジェットのキー。フローの版番号を含めるので、フローを差し替えたり
    ステップを並べ替えたりすると、前のフローのウィジェット値は使われない。"""
    return f"{name}_{st.session_state.get('flow_rev', 0)}_{idx}"


def _bump_flow_rev():
    st.session_state.flow_rev = st.session_state.get("flow_rev", 0) + 1


def _num(container, label, flow_step, idx, param, kind, default, **kwargs):
    """数値入力。表示値だけ既定値で補い、保存済みフローには書き込まない
    （書き込むのはユーザーが値を変えたときの on_change だけ）。

    フローのステップは ``flow_step``。``step`` という名前にすると、刻み幅の
    ``step=0.1`` などを ``**kwargs`` で number_input に渡せない（引数が重複する）。"""
    key = _wkey(param, idx)
    raw = flow_step.get(param)
    value = kind(default if raw is None else raw)
    container.number_input(label, value=value, key=key,
                           on_change=_update_step_param, args=(idx, param, key), **kwargs)


def _select(container, label, step, idx, param, options, default, **kwargs):
    key = _wkey(param, idx)
    current = step.get(param, default)
    index = options.index(current) if current in options else options.index(default)
    container.selectbox(label, options=options, index=index, key=key,
                        on_change=_update_step_param, args=(idx, param, key), **kwargs)


def _check(container, label, step, idx, param, default):
    key = _wkey(param, idx)
    raw = step.get(param)
    container.checkbox(label, value=(default if raw is None else raw is True), key=key,
                       on_change=_update_step_param, args=(idx, param, key))


def _text(container, label, step, idx, param):
    key = _wkey(param, idx)
    container.text_input(label, value=step.get(param) or "", key=key,
                         on_change=_update_step_param, args=(idx, param, key))


def _toggle_led_level(idx, key):
    """「明るさも設定する」の on_change: チェック時だけ level を書き込む"""
    step = st.session_state.workflow_data["steps"][idx]
    if st.session_state[key]:
        if step.get("level") is None:
            step["level"] = 12
    else:
        step.pop("level", None)


def _change_dmm_function(idx, key):
    """測定機能の on_change: 単位が変わるので固定レンジを外す（オートに戻す）

    キー付きウィジェットは既定値を変えても前の状態を保つので、「レンジを固定する」
    のチェックとレンジ欄の状態もここで合わせる（表示と実行設定を食い違わせない）。
    """
    if is_running():
        return
    step = st.session_state.workflow_data["steps"][idx]
    flow_runner.change_dmm_function(step, st.session_state[key])
    st.session_state[_wkey("fix_range", idx)] = step.get("range") is not None
    st.session_state.pop(_wkey("range", idx), None)


def _change_scope_items(idx, key):
    """測定項目の on_change: 空にはしない（スキーマが 1 つ以上を要求するので、
    空のステップは検証に通らず編集できなくなる）。空にされたら元の選択に戻す"""
    if is_running():
        return
    step = st.session_state.workflow_data["steps"][idx]
    chosen = list(st.session_state[key])
    if chosen:
        step["items"] = chosen
    else:
        st.session_state[key] = list(step.get("items") or ["vpp", "frequency"])


def _toggle_dmm_range(idx, key):
    """「レンジを固定する」の on_change: チェック時だけ range を書き込む"""
    if is_running():
        return
    step = st.session_state.workflow_data["steps"][idx]
    st.session_state.pop(_wkey("range", idx), None)   # レンジ欄は step の値から作り直す
    if st.session_state[key]:
        if step.get("range") is None:
            step["range"] = DMM_FUNCTIONS[step.get("function", "dc_voltage")][2]
    else:
        step.pop("range", None)


def render_step_params(step, idx):
    """ステップのパラメータ入力UI（popover内用）

    描画だけではフローを書き換えない: 各ウィジェットは on_change でのみ
    ``workflow_data`` を更新する。検証に通らないステップは呼び出し側で
    読み取り専用表示にしている（ここには来ない）。
    """
    action = step["action"]

    if action == "move_xyz":
        c1, c2, c3 = st.columns(3)
        _num(c1, "X", step, idx, "x", float, 0.0, label_visibility="collapsed")
        _num(c2, "Y", step, idx, "y", float, 0.0, label_visibility="collapsed")
        _num(c3, "Z", step, idx, "z", float, 0.0, label_visibility="collapsed")
        st.caption("X / Y / Z (mm)")

    elif action == "move_z":
        _num(st, "距離 (mm)", step, idx, "distance", float, 0.0)

    elif action == "move_radial":
        _num(st, "半径距離 (mm)  正=外向き / 負=内向き", step, idx, "distance", float, 20.0)

    elif action in ["rotate", "rotate_relative"]:
        label = "角度 (deg)" + (" - 相対" if action == "rotate_relative" else " - 絶対")
        c1, c2 = st.columns(2)
        _num(c1, label, step, idx, "angle", float, 0.0)
        # 速度は相対回転だけの項目（絶対回転の schema に speed は無く、書くと拒否される）
        if action == "rotate_relative":
            speed_labels = {"low": "低速", "normal": "普通", "high": "高速"}
            _select(c2, "速度", step, idx, "speed", ["low", "normal", "high"], "low",
                    format_func=lambda x: speed_labels[x])

    elif action == "move_slider":
        _num(st, "位置 (mm)", step, idx, "position", float, 500.0,
             min_value=0.0, max_value=1000.0, step=10.0)

    elif action == "move_conveyer":
        c1, c2, c3 = st.columns(3)
        _num(c1, "Index", step, idx, "index", int, 0, min_value=0, max_value=1)
        _num(c2, "速度 (mm/s)", step, idx, "speed", float, 10.0,
             min_value=0.1, max_value=200.0, step=1.0)
        _num(c3, "時間 (秒)", step, idx, "duration", float, 5.0,
             min_value=0.1, max_value=300.0, step=1.0)

    elif action == "wait":
        _num(st, "秒数", step, idx, "seconds", float, 1.0, min_value=0.1)

    elif action in ("aspirate", "dispense"):
        c1, c2 = st.columns(2)
        _num(c1, "容量 (mL)", step, idx, "volume", float, PIPETTE_MIN_VOLUME_ML,
             min_value=PIPETTE_MIN_VOLUME_ML, max_value=10.0, step=0.1)
        _num(c2, "速度", step, idx, "speed", int, 5, min_value=1, max_value=9)

    elif action == "blow_out":
        c1, c2, c3 = st.columns(3)
        _check(c1, "ホーム", step, idx, "go_home", True)
        _num(c2, "速度", step, idx, "speed", int, 1, min_value=1, max_value=9)
        _num(c3, "遅延(ms)", step, idx, "delay_ms", int, 3000,
             min_value=0, max_value=10000, step=100)

    elif action == "suction":
        _check(st, "吸着する（OFF で解放）", step, idx, "on", True)
        st.caption("MaxArm（config.yaml の type: maxarm）専用")

    elif action in ("capture_and_save", "capture_microscope"):
        _text(st, "ファイルパス（空欄で実験フォルダに自動保存）", step, idx, "file_path")

    elif action == "microscope_led":
        _check(st, "点灯する", step, idx, "on", True)
        lv_key = _wkey("set_level", idx)
        set_level = st.checkbox("明るさも設定する", value=step.get("level") is not None,
                                key=lv_key, on_change=_toggle_led_level, args=(idx, lv_key))
        if set_level and step.get("level") is not None:
            _num(st, "明るさ (0-255、出荷時 12)", step, idx, "level", int, 12,
                 min_value=0, max_value=255)

    elif action == "microscope_focus":
        _select(st, "モード", step, idx, "mode", ["auto", "position", "step"], "auto")
        mode = step.get("mode", "auto")
        if mode == "position":
            _num(st, "レンズ位置 (0-65535)", step, idx, "position", int, 1568,
                 min_value=0, max_value=65535)
        elif mode == "step":
            _select(st, "方向", step, idx, "direction", ["in", "out"], "in")
            _num(st, "回数", step, idx, "steps", int, 1, min_value=1, max_value=100)
        _num(st, "待機上限(秒)", step, idx, "timeout", float, 60.0,
             min_value=1.0, max_value=300.0, step=5.0)

    elif action == "measure_weight":
        _num(st, "測定回数", step, idx, "stabilization_count", int, 3,
             min_value=1, max_value=10)

    elif action == "tare_scale":
        _num(st, "待機時間(秒)", step, idx, "delay", float, 1.0,
             min_value=0.1, max_value=10.0, step=0.1)

    elif action == "measure_dmm":
        fn_key = _wkey("function", idx)
        functions = list(DMM_FUNCTIONS)
        current = step.get("function", "dc_voltage")
        st.selectbox("測定機能（変えると固定レンジは解除）", options=functions,
                     index=functions.index(current) if current in functions else 0,
                     key=fn_key, on_change=_change_dmm_function, args=(idx, fn_key))
        _num(st, "測定回数（中央値）", step, idx, "samples", int, 3, min_value=1, max_value=10)
        _num(st, "NPLC（積分時間）", step, idx, "nplc", float, 1.0,
             min_value=0.01, max_value=50.0, step=1.0)
        _, unit, max_range = DMM_FUNCTIONS[step.get("function", "dc_voltage")]
        rng_key = _wkey("fix_range", idx)
        fixed = st.checkbox("レンジを固定する（外すとオート）", value=step.get("range") is not None,
                            key=rng_key, on_change=_toggle_dmm_range, args=(idx, rng_key))
        if fixed and step.get("range") is not None:
            _num(st, f"レンジ ({unit}, 最大 {max_range:g})", step, idx, "range", float, max_range,
                 min_value=1e-9, max_value=max_range)
        st.caption("Keithley 2002（config.yaml の shared_devices.dmm_resource）")

    elif action == "measure_lockin":
        _num(st, "読み取り回数（中央値）", step, idx, "samples", int, 5, min_value=1, max_value=100)
        _num(st, "読み取り間隔(秒)", step, idx, "interval", float, 0.0,
             min_value=0.0, max_value=60.0, step=0.1)
        st.caption("NF LI5640（config.yaml の shared_devices.lockin_resource）。感度・時定数・"
                   "表示は前面パネルで設定し、整定を待ってから実行してください")

    elif action == "set_lockin_oscillator":
        _num(st, "周波数 (Hz)", step, idx, "frequency", float, 100.0,
             min_value=0.0005, max_value=105000.0, step=1.0, format="%.4f")
        _num(st, "振幅 (Vrms)", step, idx, "amplitude", float, 0.1,
             min_value=0.0, max_value=5.0, step=0.001, format="%.4f")
        _num(st, "設定後の待機(秒)", step, idx, "settle", float, 0.0,
             min_value=0.0, max_value=600.0, step=0.5)
        st.caption("NF LI5640 の OSC OUT を設定します。config.yaml の shared_devices."
                   "lockin_osc_max_vrms が 0（既定）なら実行前に拒否されます。振幅は接続時の"
                   "レンジのまま、ランの終わりに接続時の値へ戻ります")

    elif action == "measure_scope":
        _select(st, "チャネル", step, idx, "channel", [1, 2, 3, 4], 1)
        items_key = _wkey("items", idx)
        st.multiselect("測定項目（1〜10 個）", options=list(SCOPE_ITEMS),
                       default=list(step.get("items") or ["vpp", "frequency"]),
                       max_selections=10, key=items_key,
                       on_change=_change_scope_items, args=(idx, items_key))
        _num(st, "読み取り回数（中央値）", step, idx, "samples", int, 3, min_value=1, max_value=20)
        _num(st, "読み取り間隔(秒)", step, idx, "interval", float, 0.5,
             min_value=0.0, max_value=60.0, step=0.1)
        st.caption("Agilent 1000 シリーズ（config.yaml の shared_devices.scope_resource）。"
                   "垂直・水平・トリガは前面パネルで設定（エッジトリガ・時間軸 MAIN のみ）")

    elif action == "loop_start":
        _num(st, "繰り返し回数", step, idx, "count", int, 10, min_value=1, max_value=1000)
        st.caption(f"ループID: {step.get('loop_id', '')}")

    elif action == "loop_end":
        st.caption(f"ループID: {step.get('loop_id', '')}")
        st.info("対応する「ループ開始」と連動します")


def render_step_card(step, idx, total, step_error=None):
    """ステップカードを描画（シンプル版：カード + メニューpopover）

    ``step_error`` があるステップ（スキーマ検証に通らない）は、パラメータを
    読み取り専用で表示する。描画のために既定値を書き込んだり型変換したり
    しないため、壊れた値が黙って「直されて」実行されることはない。
    """
    action = step.get("action") if isinstance(step, dict) else None
    config = ACTION_CONFIG.get(action, {"icon": "❓", "label": escape(str(action))})
    is_shared = action in SHARED_DEVICE_ACTIONS
    is_loop = action in ["loop_start", "loop_end"]

    # スタイル決定
    if is_loop:
        bg_color = LOOP_COLORS["bg"]
        border_color = LOOP_COLORS["border"]
        bracket_color = LOOP_COLORS["bracket"]
        bracket = "┌" if action == "loop_start" else "└"
        robot_badge = ""
    elif is_shared:
        bg_color = SHARED_COLOR["bg"]
        border_color = SHARED_COLOR["border"]
        robot_badge = '<span style="float:right; font-size:0.9em; color:#666;">📡 共有</span>'
    else:
        r_id = step.get("robot_id", 1)
        robot_color = ROBOT_COLORS.get(r_id, ROBOT_COLORS[1])
        r_id = escape(str(r_id))
        bg_color = robot_color["bg"]
        border_color = robot_color["border"]
        robot_badge = f'<span style="float:right; font-size:0.9em;">{robot_color["label"]} R{r_id}</span>'

    # パラメータサマリー取得（壊れたステップでも描画を止めない）
    if step_error:
        param_summary = "⚠️ 検証エラー（読み取り専用）"
    else:
        try:
            param_summary = get_param_summary(step)
        except Exception:  # noqa: BLE001
            param_summary = ""

    # カード本体（ループ用とそれ以外で分岐）
    if is_loop:
        st.markdown(f"""
            <div style="background-color: {bg_color};
                        border-left: 6px solid {border_color};
                        padding: 8px 12px;
                        border-radius: 6px;
                        margin-bottom: 4px;
                        display: flex;
                        align-items: center;">
                <span style="font-size: 1.5em; font-weight: bold; color: {bracket_color}; margin-right: 8px;">{bracket}</span>
                <div>
                    <div style="font-weight: bold;">
                        {config['icon']} {config['label']}
                        <span style="color:#888; font-size:0.8em; margin-left:8px;">#{idx+1}</span>
                    </div>
                    {f'<div style="color:#000; font-size:1.05em;">{param_summary}</div>' if param_summary else ''}
                </div>
            </div>
        """, unsafe_allow_html=True)
    else:
        st.markdown(f"""
            <div style="background-color: {bg_color};
                        border-left: 4px solid {border_color};
                        padding: 10px 12px;
                        border-radius: 6px;
                        margin-bottom: 4px;">
                <div style="font-weight: bold; font-size: 1.1em;">
                    {config['icon']} {config['label']}
                    <span style="color:#888; font-size:0.8em; margin-left:8px;">#{idx+1}</span>
                    {robot_badge}
                </div>
                {f'<div style="color:#000; font-size:1.05em; margin-top:4px;">{param_summary}</div>' if param_summary else ''}
            </div>
        """, unsafe_allow_html=True)

    running = is_running()
    # メニューpopover（全操作を収納）
    with st.popover("⋮ メニュー", use_container_width=True):
        st.markdown(f"**{config['icon']} {config['label']} #{idx+1}**")

        # 詳細設定（一番上に配置）
        st.markdown("**詳細設定**")
        if step_error:
            st.error(step_error)
            st.caption("このステップは検証に通らないため編集できません（読み取り専用）。"
                       "JSON を直して読み込み直すか、削除してください。")
            st.json(step)
        else:
            try:
                render_step_params(step, idx)
            except Exception as e:  # noqa: BLE001 - 表示の失敗で他の操作を止めない
                st.error(f"パラメータを表示できません: {e}")
                st.json(step)

        # 順序変更
        st.divider()
        st.markdown("**順序変更**")
        col_up, col_down = st.columns(2)
        with col_up:
            st.button("↑ 上へ", key=_wkey("up", idx), disabled=(idx == 0 or running),
                      use_container_width=True, on_click=move_step, args=(idx, -1))
        with col_down:
            st.button("↓ 下へ", key=_wkey("down", idx), disabled=(idx == total - 1 or running),
                      use_container_width=True, on_click=move_step, args=(idx, 1))

        # ロボット選択（共有デバイス・ループ以外）
        if not is_shared and not is_loop and not step_error:
            st.divider()
            st.markdown("**ロボット割当**")
            current_robot = step.get("robot_id", 1)
            robot_options = [f"{c['label']} {c['name']}" for c in ROBOT_COLORS.values()]
            radio_key = _wkey("robot_select", idx)
            st.radio(
                "ロボット選択",
                robot_options,
                index=(current_robot - 1) if current_robot in ROBOT_COLORS else 0,
                disabled=running,
                horizontal=True,
                key=radio_key,
                label_visibility="collapsed",
                on_change=_update_robot_id,
                args=(idx, radio_key)
            )

        # 削除ボタン（赤色）
        st.divider()
        st.markdown('<div class="delete-btn">', unsafe_allow_html=True)
        st.button("🗑 削除", key=_wkey("del", idx), use_container_width=True,
                  disabled=running, on_click=remove_step, args=(idx,))
        st.markdown('</div>', unsafe_allow_html=True)


def render_empty_state():
    """空の状態のUI - AI生成を促す"""
    st.markdown("---")

    # 中央寄せのコンテナ
    col1, col2, col3 = st.columns([1, 6, 1])
    with col2:
        st.markdown(
            """
            <div style="text-align: center; padding: 20px 0;">
                <span style="font-size: 3em;">🤖</span>
            </div>
            """,
            unsafe_allow_html=True
        )
        st.markdown("### ワークフローを作成しましょう")

        # プリセット選択
        preset_key = None
        if PRESET_TEMPLATES:
            preset_labels = {"": "選択なし（自由入力）"}
            preset_labels.update({k: v["label"] for k, v in PRESET_TEMPLATES.items()})
            selected = st.selectbox(
                "テンプレート",
                options=list(preset_labels.keys()),
                format_func=lambda k: preset_labels[k],
                key="preset_selector",
                label_visibility="collapsed",
            )
            if selected:
                preset_key = selected
                if st.session_state.get("_last_preset") != preset_key:
                    st.session_state["_last_preset"] = preset_key
                    st.session_state["empty_state_prompt"] = ""
                    st.rerun()

        # 音声入力結果の転記（text_area描画前に実行）
        _apply_voice_result("empty_state", "empty_state_prompt")

        # AI入力エリア
        ai_prompt = st.text_area(
            "やりたいことを入力",
            placeholder="変更したいパラメータを入力してください（例: 量は3mlで速度は低速）" if preset_key else "例: robot1を50mm下降させて、50ml吸引してホームポジションに戻る",
            height=200,
            label_visibility="collapsed",
            key="empty_state_prompt"
        )

        running = is_running()
        btn_col, mic_col = st.columns([5, 1], vertical_alignment="bottom")
        generate_clicked = btn_col.button("✨ AIで生成する", type="primary", use_container_width=True,
                                          key="empty_state_generate", disabled=running)
        with mic_col:
            render_voice_input("empty_state")

        if generate_clicked and not running:
            if preset_key and not ai_prompt.strip():
                # プリセット選択でパラメータ変更なし → テンプレートをそのまま使用
                if set_workflow(PRESET_TEMPLATES[preset_key]["template"], "プリセット"):
                    st.rerun()
                st.error(st.session_state.error_message)
            elif ai_prompt.strip():
                if LLM_AVAILABLE:
                    with st.spinner("生成中..."):
                        try:
                            template = PRESET_TEMPLATES[preset_key]["template"] if preset_key else None
                            llm_input = PRESET_TEMPLATES[preset_key]["description"] + "\n\n変更指示: " + ai_prompt if preset_key else ai_prompt
                            result = generate_workflow_from_prompt(llm_input, template=template)
                        except Exception as e:
                            # 失敗したら何も読み込まない（テンプレートを「生成結果」として
                            # 載せることはしない）。入力した指示はそのまま残る。
                            st.error(f"生成に失敗しました（フローは変更していません。指示を直して再実行できます）: {e}")
                        else:
                            if set_workflow(result, "AI生成"):
                                st.rerun()
                            st.error(st.session_state.error_message)
                else:
                    st.warning("AI生成を使うには .env に GEMINI_API_KEY（または LLM_MODE=openai_compatible と LLM_BASE_URL）を設定してください")
            else:
                st.warning("やりたいことを入力してください")

        st.markdown("---")
        st.markdown(
            """
            <div style="text-align: center; color: #888;">
                または 👈 サイドバーから手動でブロックを追加
            </div>
            """,
            unsafe_allow_html=True
        )

    st.markdown("---")


def render_rejected_flow():
    """検証に通らず読み込まなかったフローを、読み取り専用で表示する"""
    rejected = st.session_state.get("rejected_flow")
    if not rejected:
        return
    with st.expander(f"⚠️ 読み込まなかったフロー（{rejected['source']}・読み取り専用）", expanded=False):
        st.caption("検証に通らなかったため、キャンバスには載せていません。"
                   "JSON を修正してから読み込み直してください。")
        st.code(rejected["error"], language="text")
        st.json(rejected["data"])
        if st.button("この表示を閉じる", key="dismiss_rejected_flow"):
            st.session_state.rejected_flow = None
            st.rerun()


def render_workflow_canvas():
    """ワークフローキャンバス（時系列順、左右で区別、ループ対応）"""
    render_rejected_flow()
    data = st.session_state.workflow_data
    steps = data.get("steps") if isinstance(data, dict) else None
    if not isinstance(steps, list):
        st.error("フローに steps の一覧がありません（読み取り専用）")
        st.json(data)
        return

    if not steps:
        render_empty_state()
        return

    # ステップ単位のスキーマ検証（通らないステップは読み取り専用で描画する）
    step_errors = flow_runner.step_errors(steps)

    # ヘッダー
    col_h1, col_h2 = st.columns(2)
    with col_h1:
        st.markdown(f"**{ROBOT_COLORS[1]['label']} Robot 1**")
    with col_h2:
        st.markdown(f"**{ROBOT_COLORS[2]['label']} Robot 2**")

    # アクティブなループを追跡（インデント計算用）
    active_loops = []

    # 時系列順にステップを表示（左右で区別）
    for idx, step in enumerate(steps):
        if not isinstance(step, dict):
            st.error(f"#{idx + 1}: ステップが JSON オブジェクトではありません（読み取り専用）: {step!r}")
            continue
        action = step.get("action")
        step_error = step_errors.get(idx)
        is_shared = action in SHARED_DEVICE_ACTIONS
        is_loop = action in ["loop_start", "loop_end"]
        robot_id = step.get("robot_id", 1) if not is_shared and not is_loop else None

        # loop_endの場合は先にスタックから削除（インデント調整）
        if action == "loop_end":
            loop_id = step.get("loop_id")
            if loop_id in active_loops:
                active_loops.remove(loop_id)

        # インデントレベル計算
        indent_level = len(active_loops)
        indent_margin = indent_level * 20  # 20px per level

        # インデント適用
        if indent_margin > 0:
            st.markdown(f'<div style="margin-left: {indent_margin}px;">', unsafe_allow_html=True)

        if is_loop:
            # ループ: 全幅で表示
            render_step_card(step, idx, len(steps), step_error)
        elif is_shared:
            # 共有デバイス: 中央配置
            col_l, col_c, col_r = st.columns([1, 1, 1])
            with col_c:
                render_step_card(step, idx, len(steps), step_error)
        elif robot_id == 1:
            # Robot 1: 左側
            col1, col2, col3 = st.columns(3)
            with col1:
                render_step_card(step, idx, len(steps), step_error)
        elif robot_id == 2:
            # Robot 2: 中央
            col1, col2, col3 = st.columns(3)
            with col2:
                render_step_card(step, idx, len(steps), step_error)
        else:
            # Robot 3 / Robot 4 (MaxArm): 右側
            col1, col2, col3 = st.columns(3)
            with col3:
                render_step_card(step, idx, len(steps), step_error)

        if indent_margin > 0:
            st.markdown('</div>', unsafe_allow_html=True)

        # loop_startの場合はスタックに追加
        if action == "loop_start":
            loop_id = step.get("loop_id")
            active_loops.append(loop_id)


def render_settings_panel():
    """設定パネル"""
    st.markdown("##### ⚙️ 設定")

    running = is_running()

    with st.expander("🔧 接続ポート", expanded=False):
        st.caption(
            "既定値は config.yaml（無ければ config.example.yaml）です。"
            "ここでの変更はこのセッションだけに効きます。"
            "実際に初期化されるのはフローが使うロボット・デバイスだけです。"
        )
        for robot_id in ROBOT_COLORS:
            color = ROBOT_COLORS[robot_id]
            config = st.session_state.robot_configs.get(robot_id)
            if config is None:
                continue
            st.markdown(f"**{color['label']} {color['name']}**")
            if config.get("type") == "maxarm":
                # MaxArm: 吸着ノズルのみ（ピペットなし）。種別は config.yaml で決める
                config["maxarm_port"] = st.text_input(
                    "MaxArm Port", value=config.get("maxarm_port", ""),
                    key=f"r{robot_id}_mp", disabled=running,
                    help="Hiwonder MaxArm（CH340）のシリアルポート。"
                         "空のままでは実機ランを開始できません")
                continue
            config["dobot_port"] = st.text_input(
                "Dobot Port", value=config.get("dobot_port", ""),
                key=f"r{robot_id}_dp", disabled=running)
            config["use_picus2"] = st.checkbox(
                "Picus2 あり", value=config.get("use_picus2", False),
                key=f"r{robot_id}_picus2", disabled=running)
            if config["use_picus2"]:
                config["picus2_port"] = st.text_input(
                    "Picus2 Port", value=config.get("picus2_port", ""),
                    key=f"r{robot_id}_pp", disabled=running)

    with st.expander("📡 共有デバイス", expanded=False):
        config = st.session_state.shared_config
        config["scale_port"] = st.text_input(
            "天秤 Port", value=config["scale_port"], key="scale_port", disabled=running)
        config["camera_index"] = st.number_input(
            "カメラ index", value=int(config.get("camera_index", 0)), min_value=0,
            step=1, key="camera_index", disabled=running)

    if st.button("↺ config.yaml の値に戻す", use_container_width=True, disabled=running):
        defaults = flow_runner.default_device_settings()
        st.session_state.robot_configs = defaults["robots"]
        st.session_state.shared_config = defaults["shared"]
        # ウィジェットに残った入力値が戻した値を上書きしないよう、その状態も捨てる
        for _rid in defaults["robots"]:
            for _suffix in ("_dp", "_picus2", "_pp", "_mp"):
                st.session_state.pop(f"r{_rid}{_suffix}", None)
        for _key in ("scale_port", "camera_index"):
            st.session_state.pop(_key, None)
        st.rerun()

    st.divider()
    uploaded_file = st.file_uploader("📂 JSON読込", type="json", label_visibility="collapsed",
                                     disabled=running,
                                     help="検証に通ったフローだけがキャンバスに読み込まれます")
    if uploaded_file and not running:
        file_id = uploaded_file.file_id
        if st.session_state.get("_loaded_file_id") != file_id:
            st.session_state._loaded_file_id = file_id
            try:
                data = json.load(uploaded_file)
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                st.error(f"JSON を読めません: {e}")
            else:
                if set_workflow(data, "読み込んだJSON"):
                    st.success("読込完了")
                else:
                    st.error("検証に失敗したため読み込みませんでした（詳細はメイン画面）")

    with st.popover("📄 JSON確認"):
        st.json(st.session_state.workflow_data)


# ==========================================
# 8. メインレイアウト
# ==========================================
st.title("🧩 Visual Automation Controller")

# カスタムCSS
st.markdown("""
<style>
/* 実行パネル */
.execution-panel {
    padding: 8px 0;
    border-bottom: 2px solid #e0e0e0;
    margin-bottom: 12px;
}

/* 緊急停止ボタン（赤色） - execution-panel内の最初のカラムのボタン */
.execution-panel [data-testid="column"]:first-child button {
    background-color: #ffebee !important;
    border: 2px solid #f44336 !important;
    color: #c62828 !important;
}
.execution-panel [data-testid="column"]:first-child button:hover {
    background-color: #ffcdd2 !important;
    border-color: #d32f2f !important;
}

/* 削除ボタン（赤色テキスト） */
.delete-btn button {
    color: #c62828 !important;
    border-color: #ef9a9a !important;
}
.delete-btn button:hover {
    background-color: #ffebee !important;
}
</style>
""", unsafe_allow_html=True)

# サイドバー（ツール・設定）
with st.sidebar:
    render_toolbox()
    st.divider()
    render_settings_panel()

# メインエリア
# エラーメッセージ表示
if st.session_state.error_message:
    st.error(st.session_state.error_message)
    st.session_state.error_message = None

# 進捗の取り込み（バックグラウンド実行 → ログ・進捗・完了状態）
drain_runner_events()

# 実行パネル（停止 / 検証 / 実行）はキャンバスより先に描画する: キャンバスの
# 描画中に例外が出ても、停止ボタンは必ず画面に残る。
render_execution_panel()

# ワークフロー - ヘッダーとAIボタン
col_wf_title, col_ai_btn = st.columns([3, 1])
with col_wf_title:
    st.markdown("##### 📋 ワークフロー")
with col_ai_btn:
    _ai_clicked = st.button("✨ AIで新規作成", use_container_width=True, disabled=is_running())
    _reopen = st.session_state.pop("_reopen_dialog", False)
    if (_ai_clicked or _reopen) and not is_running():
        ai_workflow_dialog()

try:
    render_workflow_canvas()
except Exception as _canvas_error:  # noqa: BLE001 - 停止ボタンは上で描画済み
    st.error(f"ワークフローの描画に失敗しました（読み取り専用で表示します）: {_canvas_error}")
    st.json(st.session_state.workflow_data)

# ログエリア（固定高さ）
st.markdown("##### 📜 ログ")
with st.container(height=150):
    if st.session_state.log_messages:
        st.code("\n".join(st.session_state.log_messages), language="bash")
    else:
        st.code("待機中...", language="bash")


# ==========================================
# 9. 実行中のポーリング
# ==========================================
# 実行そのものは runner のバックグラウンドスレッドが最後まで走らせる。
# ここでは一定間隔で rerun して、キューに溜まった進捗を描画するだけ。
_runner = st.session_state.runner
if _runner is not None and (_runner.is_running() or not _runner.events.empty()):
    time.sleep(0.4)
    st.rerun()
