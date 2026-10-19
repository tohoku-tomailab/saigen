"""
L2層: 実験ログ管理クラス（ExperimentLogger）

実験（ワークフロー）1回の実行ごとに専用フォルダを作成し、
トレーサビリティ（再現性確保）のために以下を残す:

    logs/<日付>/<実験名>_<タイムスタンプ>/
        run.log          全ログ（コンソール出力と同じ内容をファイルにも保存）
        <入力ファイル>   実行に使った入力ファイルのコピー（JSON/CSV、元のファイル名を保持）
        metadata.json    実行メタデータ（開始/終了時刻・使用ロボット・結果ステータス等）
        measurements.csv 各ステップの構造化記録（重量・DMM 測定値・画像パス・成否・所要時間）
        summary.md       天秤・DMM の測定値と画像を時系列でひも付けた人間可読レポート
        images/          このワークフローで撮影した画像（自動でこのフォルダへ束ねる）

実行モード（"mock" / "real"）は metadata.json の ``mode``、measurements.csv の
``mode`` 列、summary.md の概要に必ず残す（Mock の記録を実測と取り違えないため）。
同じ秒に同名の実験が始まってもフォルダは共有せず、``_2``, ``_3`` ... を付けて
別フォルダにする（既存フォルダへは決して書き込まない）。

設計思想:
- src/flow/run_flow.py から利用する。ロガーへの FileHandler 着脱もここで面倒を見る。
- 各モジュール（executor / shared_devices / webcam 等）のログも漏らさず拾うため、
  ルートロガーに FileHandler を装着する。
"""

import csv
import json
import logging
import math
import os
import re
import shutil
import time
from datetime import datetime


def _full_precision(value) -> str:
    """DMM の値を丸めずに文字列化する（8.5 桁の分解能を CSV で失わない）"""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return ""
    return repr(float(value))


def _json_cell(value) -> str:
    """dict / list を CSV の 1 セルに入れる JSON 文字列にする（空なら ""）"""
    if value is None or value == {} or value == []:
        return ""
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


#: 実行モードの表記（metadata / CSV / summary で共通）
MODES = ("mock", "real")
_MODE_LABELS = {"mock": "MOCK（シミュレーション・実機なし）", "real": "REAL（実機）"}


def _normalize_mode(mode) -> str:
    if mode is None:
        return "unspecified"
    if isinstance(mode, bool):  # mock=True / False を直接渡された場合
        return "mock" if mode else "real"
    text = str(mode).strip().lower()
    if text not in MODES:
        raise ValueError(f"mode は {MODES} のいずれか: {mode!r}")
    return text


def _make_unique_dir(path: str) -> str:
    """``path`` を新規作成して返す。既にあれば ``_2``, ``_3`` ... を付ける。

    ``exist_ok=True`` は使わない: 同じ秒に始まった 2 つのランが 1 つのフォルダ
    （run.log・measurements.csv）を共有してしまうため。
    """
    parent = os.path.dirname(path)
    os.makedirs(parent, exist_ok=True)   # 日付フォルダは共有してよい
    candidate, n = path, 1
    while True:
        try:
            os.makedirs(candidate)
            return candidate
        except FileExistsError:
            n += 1
            if n > 9999:
                raise
            candidate = f"{path}_{n}"


def _sanitize(name: str) -> str:
    """フォルダ名に使えない文字を除去（日本語はそのまま残す）"""
    name = name.strip() or "experiment"
    # Windows で禁止される文字と制御文字を置換
    name = re.sub(r'[\\/:*?"<>|\r\n\t]', "_", name)
    return name[:80]  # 長すぎるフォルダ名を防ぐ


class ExperimentLogger:
    """実験1回分のログ・成果物を専用フォルダにまとめて残す"""

    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"

    def __init__(self, name: str, description: str, source_json_path: str,
                 base_dir: str = "logs", mode=None):
        """
        Args:
            mode: 実行モード "mock" / "real"（bool の mock フラグも可）。
                省略時は "unspecified" と記録される。
        """
        self.name = name
        self.mode = _normalize_mode(mode)
        self.recording = None      # センサー録画の所有情報（record_recording）
        self.description = description or ""
        self.source_json_path = source_json_path

        # 実験フォルダの作成（logs/<日付>/<名前>_<タイムスタンプ>/）
        # 日付ごとにフォルダを切り、その中に実行ごとのフォルダを作る
        self.started_at = datetime.now()
        date_dir = self.started_at.strftime("%Y-%m-%d")
        timestamp = self.started_at.strftime("%Y%m%d_%H%M%S")
        folder = f"{_sanitize(name)}_{timestamp}"
        self.dir = _make_unique_dir(os.path.abspath(os.path.join(base_dir, date_dir, folder)))
        self.images_dir = os.path.join(self.dir, "images")
        os.makedirs(self.images_dir)

        # 経過時間計測用（壁時計とは別に単調増加クロックを使う）
        self._t0 = time.monotonic()

        # 構造化記録の蓄積先
        self.records = []          # 全ステップの記録
        self.events = []           # 重量測定・画像撮影など特筆すべきイベント
        self.step_diagnostics = []  # 失敗したステップの診断データ（metadata.json へ）
        self.lockin_settings = []   # 成功したロックイン読み取りのパネル設定（metadata.json へ）
        self.scope_readings = []    # 成功したオシロ読み取りの設定と応答文字列（metadata.json へ）
        self.lockin_oscillator = []  # LI5640 の内部発振器の設定ステップ（前後・参照信号源）
        self.output_restore = None   # 出力の復元結果（LI5640 の発振器。record_output_restore）
        self.robots_used = set()
        self.picus2_robots = set()
        self.robot_types = {}      # robot_id -> "dobot" / "maxarm"
        self.status = "running"
        self.error = None
        self.finished_at = None

        # ファイルパス
        self.log_path = os.path.join(self.dir, "run.log")
        self.csv_path = os.path.join(self.dir, "measurements.csv")
        self.metadata_path = os.path.join(self.dir, "metadata.json")
        self.summary_path = os.path.join(self.dir, "summary.md")
        # 入力ファイルのコピー先（元のファイル名を保持。JSONでもCSVでも対応）
        copy_name = os.path.basename(source_json_path) or "input"
        self.workflow_copy_path = os.path.join(self.dir, copy_name)

        # 実行に使った入力ファイルを即コピー（実行中に消えても記録が残るように）
        try:
            shutil.copyfile(source_json_path, self.workflow_copy_path)
        except Exception:
            pass

        # ルートロガーに FileHandler を装着（全モジュールのログを拾う）
        self._file_handler = logging.FileHandler(self.log_path, encoding="utf-8")
        self._file_handler.setLevel(logging.INFO)
        self._file_handler.setFormatter(logging.Formatter(self.LOG_FORMAT))
        logging.getLogger().addHandler(self._file_handler)

    # ------------------------------------------------------------------
    # 記録 API
    # ------------------------------------------------------------------
    def record_resources(self, robot_ids, picus2_robots, robot_types=None):
        """使用リソースをメタデータに反映

        Args:
            robot_ids: 使用するロボット ID
            picus2_robots: ピペットを使うロボット ID
            robot_types: robot_id → 種別（"dobot" / "maxarm"）。省略時は
                使用ロボットすべて "dobot"
        """
        self.robots_used = set(robot_ids)
        self.picus2_robots = set(picus2_robots)
        types = dict(robot_types or {})
        self.robot_types = {int(rid): types.get(rid, types.get(str(rid), "dobot"))
                            for rid in self.robots_used}

    def record_recording(self, info: dict):
        """センサー CSV 録画の情報を残す（``started_by_this_run`` など）。

        このランが開始していない録画（/api/start が 409）は止めない、という
        判断の根拠を後から確認できるようにする。
        """
        self.recording = dict(info) if info is not None else None

    def record_step(self, index, total, action, robot_id, iteration,
                    status, duration, result=None, error=None, diagnostics=None):
        """1ステップの実行結果を記録

        Args:
            index: ステップ番号（1始まり、ループ展開後）
            total: 総ステップ数
            action: アクション名
            robot_id: ロボットID（共有デバイスは None）
            iteration: ループ反復番号（ループ外は None）
            status: "ok" / "error" / "aborted"（Ctrl+C などで中断したステップ）
            duration: 所要秒数
            result: execute_step の戻り値（{"weight":..} / {"image_path":..} /
                {"dmm_value":.., "dmm_unit":..} 等）
            error: 例外メッセージ（失敗時）
            diagnostics: 失敗の診断データ（例外の ``diagnostics`` 属性。ロックインの
                設定変化の前後・差分、捨てたサンプルの理由など）。CSV の diagnostics 列、
                時系列イベント、metadata.json の step_diagnostics に残す
        """
        now = datetime.now()
        elapsed = time.monotonic() - self._t0
        result = result or {}
        weight = result.get("weight")
        image_path = result.get("image_path")
        nominal_time = result.get("nominal_time_s")
        focus_position = result.get("focus_position")
        focus_converged = result.get("focus_converged")
        dmm_value = result.get("dmm_value")
        # 撮影ステップが「画像なし」で成功扱いになるのを防ぐ（executor も例外に
        # するが、戻り値だけを渡す呼び出し側があっても記録を誤らせない）
        if status == "ok" and "image_path" in result and not image_path:
            status = "error"
            error = error or "撮影に失敗しました（画像パスが返りませんでした）"

        record = {
            "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": self.mode,
            "elapsed_s": round(elapsed, 2),
            "step_index": index,
            "total_steps": total,
            "iteration": iteration if iteration is not None else "",
            "action": action,
            "robot_id": robot_id if robot_id is not None else "",
            "status": status,
            "duration_s": round(duration, 3),
            "weight_g": f"{weight:.3f}" if isinstance(weight, (int, float)) else "",
            "nominal_time_s": f"{nominal_time:.2f}" if isinstance(nominal_time, (int, float)) else "",
            "image_path": image_path or "",
            "focus_position": str(focus_position) if isinstance(focus_position, int) else "",
            "focus_converged": ("" if focus_converged is None else ("True" if focus_converged else "False")),
            "dmm_value": _full_precision(dmm_value),
            "dmm_unit": result.get("dmm_unit") or "",
            "dmm_function": result.get("dmm_function") or "",
            "dmm_range": _full_precision(result.get("dmm_range")),
            "dmm_nplc": _full_precision(result.get("dmm_nplc")),
            "dmm_valid_samples": (str(result["dmm_valid_samples"])
                                  if isinstance(result.get("dmm_valid_samples"), int) else ""),
            "lockin_data1": _full_precision(result.get("lockin_data1")),
            "lockin_data1_kind": result.get("lockin_data1_kind") or "",
            "lockin_data1_unit": result.get("lockin_data1_unit") or "",
            "lockin_data2": _full_precision(result.get("lockin_data2")),
            "lockin_data2_kind": result.get("lockin_data2_kind") or "",
            "lockin_data2_unit": result.get("lockin_data2_unit") or "",
            "lockin_freq": _full_precision(result.get("lockin_freq")),
            "lockin_sensitivity": result.get("lockin_sensitivity") or "",
            "lockin_time_constant": result.get("lockin_time_constant") or "",
            "lockin_valid_samples": (str(result["lockin_valid_samples"])
                                     if isinstance(result.get("lockin_valid_samples"), int) else ""),
            "lockin_invalid": _json_cell(result.get("lockin_invalid")),
            "lockin_settings": _json_cell(result.get("lockin_settings")),
            "scope_channel": (str(result["scope_channel"])
                              if isinstance(result.get("scope_channel"), int) else ""),
            "scope_values": _json_cell(result.get("scope_values")),
            "scope_units": _json_cell(result.get("scope_units")),
            "scope_valid": _json_cell(result.get("scope_valid")),
            "scope_invalid": _json_cell(result.get("scope_invalid")),
            "scope_settings": _json_cell(result.get("scope_settings")),
            "lockin_osc_freq_hz": _full_precision(result.get("lockin_osc_freq_hz")),
            "lockin_osc_ampl_vrms": _full_precision(result.get("lockin_osc_ampl_vrms")),
            "lockin_osc_range": (str(result["lockin_osc_range"])
                                 if isinstance(result.get("lockin_osc_range"), int) else ""),
            "lockin_osc_settle_s": _full_precision(result.get("lockin_osc_settle_s")),
            "lockin_osc_settled_s": _full_precision(result.get("lockin_osc_settled_s")),
            "diagnostics": _json_cell(diagnostics),
            "error": error or "",
        }
        self.records.append(record)
        if diagnostics:
            self.step_diagnostics.append({"step_index": index, "action": action,
                                          "iteration": record["iteration"],
                                          "error": error or "", "diagnostics": diagnostics})
        if status == "ok" and isinstance(result.get("scope_values"), dict):
            # 設定と応答文字列はステップごとに構造のまま残す（未確認の応答形式の調査用）
            self.scope_readings.append({"step_index": index, "iteration": record["iteration"],
                                        "channel": result.get("scope_channel"),
                                        "settings": result.get("scope_settings"),
                                        "settings_after": result.get("scope_settings_after"),
                                        "raw": result.get("scope_raw")})
        if status == "ok" and record["lockin_osc_freq_hz"]:
            self.lockin_oscillator.append({"step_index": index, "iteration": record["iteration"],
                                           "before": result.get("lockin_osc_before"),
                                           "after": {"freq_hz": result.get("lockin_osc_freq_hz"),
                                                     "ampl_vrms": result.get("lockin_osc_ampl_vrms"),
                                                     "range": result.get("lockin_osc_range")},
                                           "rsrc": result.get("lockin_osc_rsrc"),
                                           "settle_s": result.get("lockin_osc_settle_s"),
                                           "settled_s": result.get("lockin_osc_settled_s")})
        if status == "ok" and isinstance(result.get("lockin_settings"), dict):
            # 成功した読み取りのパネル設定も構造のまま残す（CSV の JSON 文字列を解析させない）
            self.lockin_settings.append({"step_index": index, "iteration": record["iteration"],
                                         "settings": result["lockin_settings"]})

        # 特筆イベント（時系列レポート用）
        if weight is not None:
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "weight", "detail": f"{weight:.3f} g",
                "step_index": index, "iteration": record["iteration"],
            })
        if image_path:
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "image", "detail": image_path,
                "step_index": index, "iteration": record["iteration"],
            })
        if record["dmm_value"]:
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "dmm",
                "detail": f"{record['dmm_value']} {record['dmm_unit']} ({record['dmm_function']})",
                "step_index": index, "iteration": record["iteration"],
            })
        if record["lockin_data1"]:
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "lockin",
                "detail": (f"{record['lockin_data1_kind']} {record['lockin_data1']} "
                           f"{record['lockin_data1_unit']}, {record['lockin_data2_kind']} "
                           f"{record['lockin_data2']} {record['lockin_data2_unit']}, "
                           f"{record['lockin_freq']} Hz"),
                "step_index": index, "iteration": record["iteration"],
            })
        if record["lockin_osc_freq_hz"]:
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "lockin_osc",
                "detail": (f"{record['lockin_osc_freq_hz']} Hz, {record['lockin_osc_ampl_vrms']} Vrms"
                           f"（レンジ {record['lockin_osc_range']}）"),
                "step_index": index, "iteration": record["iteration"],
            })
        if record["scope_values"]:
            shown = ", ".join(f"{k} {v} {(result.get('scope_units') or {}).get(k, '')}".strip()
                              for k, v in (result.get("scope_values") or {}).items())
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "scope", "detail": f"CH{record['scope_channel']}: {shown}",
                "step_index": index, "iteration": record["iteration"],
            })
        if diagnostics:
            keys = ", ".join(sorted(diagnostics.get("settings_diff", {}))) or ""
            invalid = diagnostics.get("invalid") or {}
            parts = ([f"設定の変化: {keys}"] if keys else []) + (
                [f"捨てたサンプル: {_json_cell(invalid)}"] if invalid else [])
            self.events.append({
                "timestamp": record["timestamp"], "elapsed_s": record["elapsed_s"],
                "kind": "diagnostics",
                "detail": "; ".join(parts) or (error or "診断データあり（metadata.json）"),
                "step_index": index, "iteration": record["iteration"],
            })

    def record_output_restore(self, report):
        """切断時の出力の復元結果（LI5640 の発振器）を残す。None なら何もしない"""
        if report is not None:
            self.output_restore = report

    def next_image_path(self, index, tag: str = ""):
        """capture_and_save / capture_microscope 用の保存先パスを払い出す
        （画像をフォルダ内へ束ねる）。``tag`` はファイル名に挟む識別子
        （顕微鏡画像は "microscope"）。"""
        ts = datetime.now().strftime("%H%M%S")
        middle = f"_{tag}" if tag else ""
        filename = f"step{index:03d}{middle}_{ts}.jpg"
        return os.path.join(self.images_dir, filename)

    # ------------------------------------------------------------------
    # 終了処理
    # ------------------------------------------------------------------
    def finalize(self, status: str, error: str = None):
        """成果物（CSV / metadata.json / summary.md）を書き出してログを閉じる"""
        self.status = status
        self.error = error
        self.finished_at = datetime.now()

        try:
            self._write_csv()
            self._write_metadata()
            self._write_summary()
        finally:
            # FileHandler を必ず外す（次の実行で重複しないように）
            logging.getLogger().removeHandler(self._file_handler)
            self._file_handler.close()

    def _duration_s(self):
        if self.finished_at is None:
            return 0.0
        return (self.finished_at - self.started_at).total_seconds()

    def _write_csv(self):
        fields = ["timestamp", "mode", "elapsed_s", "step_index", "total_steps",
                  "iteration", "action", "robot_id", "status", "duration_s",
                  "weight_g", "nominal_time_s", "image_path", "focus_position",
                  "focus_converged", "dmm_value", "dmm_unit", "dmm_function",
                  "dmm_range", "dmm_nplc", "dmm_valid_samples",
                  "lockin_data1", "lockin_data1_kind", "lockin_data1_unit",
                  "lockin_data2", "lockin_data2_kind", "lockin_data2_unit",
                  "lockin_freq", "lockin_sensitivity", "lockin_time_constant",
                  "lockin_valid_samples", "lockin_invalid", "lockin_settings",
                  "scope_channel", "scope_values", "scope_units", "scope_valid",
                  "scope_invalid", "scope_settings", "lockin_osc_freq_hz",
                  "lockin_osc_ampl_vrms", "lockin_osc_range", "lockin_osc_settle_s",
                  "lockin_osc_settled_s", "diagnostics", "error"]
        with open(self.csv_path, "w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in self.records:
                writer.writerow(r)

    def _write_metadata(self):
        weights = [r for r in self.records if r["weight_g"] != ""]
        images = [r for r in self.records if r["image_path"] != ""]
        dmm_readings = [r for r in self.records if r["dmm_value"] != ""]
        lockin_readings = [r for r in self.records if r["lockin_data1"] != ""]
        scope_rows = [r for r in self.records if r["scope_values"] != ""]
        meta = {
            "name": self.name,
            "description": self.description,
            "mode": self.mode,
            "source_json": os.path.abspath(self.source_json_path),
            "started_at": self.started_at.strftime("%Y-%m-%d %H:%M:%S"),
            "finished_at": self.finished_at.strftime("%Y-%m-%d %H:%M:%S")
            if self.finished_at else None,
            "duration_s": round(self._duration_s(), 2),
            "status": self.status,
            "error": self.error,
            "total_steps": len(self.records),
            "ok_steps": sum(1 for r in self.records if r["status"] == "ok"),
            "error_steps": sum(1 for r in self.records if r["status"] == "error"),
            "aborted_steps": sum(1 for r in self.records if r["status"] == "aborted"),
            "robots_used": sorted(self.robots_used),
            "picus2_robots": sorted(self.picus2_robots),
            # JSON のキーは文字列になる: {"4": "maxarm"}
            "robot_types": {str(rid): t for rid, t in sorted(self.robot_types.items())},
            "weight_measurements": len(weights),
            "dmm_measurements": len(dmm_readings),
            "lockin_measurements": len(lockin_readings),
            # 失敗したステップの診断データ（ロックインの設定変化の前後など）
            "step_diagnostics": self.step_diagnostics,
            # 成功したロックイン読み取りごとのパネル設定（感度・時定数・表示など）
            "lockin_settings": self.lockin_settings,
            "scope_measurements": len(scope_rows),
            # 成功したオシロ読み取りごとのパネル設定と応答文字列
            "scope_readings": self.scope_readings,
            # LI5640 の内部発振器の設定ステップと、切断時の復元結果
            "lockin_oscillator": self.lockin_oscillator,
            "lockin_oscillator_restore": self.output_restore,
            "images_captured": len(images),
            "sensor_recording": self.recording,
            "output_files": {
                "log": os.path.basename(self.log_path),
                "workflow": os.path.basename(self.workflow_copy_path),
                "measurements_csv": os.path.basename(self.csv_path),
                "summary": os.path.basename(self.summary_path),
                "images_dir": "images",
            },
        }
        with open(self.metadata_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

    def _write_summary(self):
        status_label = {"completed": "✅ 完了", "failed": "❌ 失敗",
                        "aborted": "⛔ 中断", "running": "⏳ 実行中"}.get(
                            self.status, self.status)
        weights = [e for e in self.events if e["kind"] == "weight"]
        images = [e for e in self.events if e["kind"] == "image"]
        dmm_events = [e for e in self.events if e["kind"] == "dmm"]
        lockin_events = [e for e in self.events if e["kind"] == "lockin"]
        scope_events = [e for e in self.events if e["kind"] == "scope"]

        lines = []
        lines.append(f"# 実験レポート: {self.name}")
        lines.append("")
        if self.description:
            lines.append(f"> {self.description}")
            lines.append("")
        lines.append("## 概要")
        lines.append("")
        lines.append(f"- **ステータス**: {status_label}")
        lines.append(f"- **実行モード**: {_MODE_LABELS.get(self.mode, self.mode)}")
        lines.append(f"- **開始**: {self.started_at.strftime('%Y-%m-%d %H:%M:%S')}")
        if self.finished_at:
            lines.append(f"- **終了**: {self.finished_at.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"- **所要時間**: {self._duration_s():.1f} 秒")
        lines.append(f"- **総ステップ数**: {len(self.records)}"
                     f"（成功 {sum(1 for r in self.records if r['status']=='ok')} /"
                     f" 失敗 {sum(1 for r in self.records if r['status']=='error')} /"
                     f" 中断 {sum(1 for r in self.records if r['status']=='aborted')}）")
        if self.robots_used:
            lines.append(f"- **使用ロボット**: {sorted(self.robots_used)}")
            if self.robot_types:
                lines.append("- **ロボット種別**: " + ", ".join(
                    f"Robot {rid} = {t}" for rid, t in sorted(self.robot_types.items())))
        lines.append(f"- **重量測定**: {len(weights)} 回 / **撮影画像**: {len(images)} 枚"
                     + (f" / **DMM 測定**: {len(dmm_events)} 回" if dmm_events else "")
                     + (f" / **ロックイン測定**: {len(lockin_events)} 回" if lockin_events else "")
                     + (f" / **オシロ測定**: {len(scope_events)} 回" if scope_events else ""))
        if self.error:
            lines.append(f"- **エラー**: {self.error}")
        if self.output_restore is not None:
            osc_state = (self.output_restore.get("oscillator") or {}).get("state")
            label = {"not_needed": "書き込みなし（戻す必要なし）", "verified": "✅ 接続時の値へ戻した",
                     "failed": "❌ 戻せなかった（前面パネルで確認）",
                     "unverified": "⚠️ 確かめられなかった（前面パネルで確認）"}.get(osc_state, osc_state)
            lines.append(f"- **LI5640 発振器の復元**: {label}"
                         f"（切断: {self.output_restore.get('closed')}）")
        lines.append("")

        # 時系列イベント（重量・画像をひも付け）
        if self.recording is not None:
            owned = self.recording.get("started_by_this_run")
            lines.append(f"- **センサー録画**: "
                         + ("このランが開始・終了" if owned else "このランは開始していない（停止もしない）")
                         + (f"（{self.recording.get('save_dir')}）" if self.recording.get("save_dir") else ""))
            lines.append("")
        lines.append("## 測定・記録の時系列")
        lines.append("")
        if self.events:
            lines.append("| 経過(s) | 時刻 | 種別 | ステップ | 反復 | 内容 |")
            lines.append("|--------:|------|------|---------:|------|------|")
            for e in self.events:
                kind = {"weight": "⚖️ 重量", "image": "📷 画像", "dmm": "🔌 DMM",
                        "lockin": "📈 ロックイン", "scope": "📉 オシロ",
                        "lockin_osc": "🔊 発振器",
                        "diagnostics": "⚠️ 診断"}.get(e["kind"], e["kind"])
                detail = e["detail"]
                if e["kind"] == "image":
                    rel = os.path.relpath(detail, self.dir) if os.path.isabs(detail) else detail
                    detail = f"[{os.path.basename(detail)}]({rel.replace(os.sep, '/')})"
                lines.append(f"| {e['elapsed_s']:.1f} | {e['timestamp'].split(' ')[1]} "
                             f"| {kind} | {e['step_index']} | {e['iteration']} | {detail} |")
        else:
            lines.append("（重量測定・DMM 測定・ロックイン測定・オシロ測定・画像撮影はありませんでした）")
        lines.append("")

        # 重量測定の抜粋（解析しやすいように数値だけ再掲）
        if weights:
            lines.append("## 重量測定値")
            lines.append("")
            lines.append("| # | 経過(s) | 反復 | 重量(g) |")
            lines.append("|---|--------:|------|--------:|")
            for i, e in enumerate(weights, 1):
                grams = e["detail"].replace(" g", "")
                lines.append(f"| {i} | {e['elapsed_s']:.1f} | {e['iteration']} | {grams} |")
            lines.append("")

        # DMM 測定値の抜粋（丸めない値・単位・機能）
        if dmm_events:
            dmm_rows = [r for r in self.records if r["dmm_value"] != ""]
            lines.append("## DMM 測定値")
            lines.append("")
            lines.append("| # | 経過(s) | 反復 | 機能 | 値 | 単位 | レンジ | NPLC | 有効回数 |")
            lines.append("|---|--------:|------|------|---:|------|------:|-----:|------:|")
            for i, r in enumerate(dmm_rows, 1):
                lines.append(f"| {i} | {r['elapsed_s']:.1f} | {r['iteration']} | {r['dmm_function']} "
                             f"| {r['dmm_value']} | {r['dmm_unit']} | {r['dmm_range']} "
                             f"| {r['dmm_nplc']} | {r['dmm_valid_samples']} |")
            lines.append("")

        # ロックインアンプの読み取り値（丸めない値・表示の種類・単位）
        if lockin_events:
            rows = [r for r in self.records if r["lockin_data1"] != ""]
            lines.append("## ロックイン測定値")
            lines.append("")
            lines.append("| # | 経過(s) | 反復 | DATA1 | 単位 | DATA2 | 単位 | 周波数(Hz) "
                         "| 感度 | 時定数 | 有効回数 |")
            lines.append("|---|--------:|------|------:|------|------:|------|-----------:"
                         "|------|--------|------:|")
            for i, r in enumerate(rows, 1):
                lines.append(f"| {i} | {r['elapsed_s']:.1f} | {r['iteration']} "
                             f"| {r['lockin_data1_kind']} {r['lockin_data1']} | {r['lockin_data1_unit']} "
                             f"| {r['lockin_data2_kind']} {r['lockin_data2']} | {r['lockin_data2_unit']} "
                             f"| {r['lockin_freq']} | {r['lockin_sensitivity']} "
                             f"| {r['lockin_time_constant']} | {r['lockin_valid_samples']} |")
            lines.append("")

        # オシロスコープの自動測定値（項目ごとの中央値・単位・有効回数）
        if scope_events:
            lines.append("## オシロ測定値")
            lines.append("")
            lines.append("| # | 経過(s) | 反復 | CH | 項目 | 値 | 単位 | 有効回数 |")
            lines.append("|---|--------:|------|---:|------|---:|------|------:|")
            n = 0
            for r in self.records:
                if r["scope_values"] == "":
                    continue
                values = json.loads(r["scope_values"])
                units = json.loads(r["scope_units"] or "{}")
                valid = json.loads(r["scope_valid"] or "{}")
                for item, value in values.items():
                    n += 1
                    lines.append(f"| {n} | {r['elapsed_s']:.1f} | {r['iteration']} "
                                 f"| {r['scope_channel']} | {item} | {value!r} "
                                 f"| {units.get(item, '')} | {valid.get(item, '')} |")
            lines.append("")

        # 全ステップの実行記録
        lines.append("## 全ステップ実行記録")
        lines.append("")
        lines.append("| # | 経過(s) | アクション | Robot | 反復 | 状態 | 所要(s) | 備考 |")
        lines.append("|---|--------:|-----------|-------|------|------|--------:|------|")
        for r in self.records:
            note = ""
            if r["weight_g"]:
                note = f"{r['weight_g']} g"
            elif r["image_path"]:
                note = os.path.basename(r["image_path"])
            elif r.get("dmm_value"):
                note = f"{r['dmm_value']} {r['dmm_unit']}"
            elif r.get("scope_values"):
                note = f"CH{r['scope_channel']} {r['scope_values']}"
            elif r.get("lockin_osc_freq_hz"):
                note = f"{r['lockin_osc_freq_hz']} Hz, {r['lockin_osc_ampl_vrms']} Vrms"
            elif r.get("lockin_data1"):
                note = (f"{r['lockin_data1_kind']} {r['lockin_data1']} {r['lockin_data1_unit']}, "
                        f"{r['lockin_data2_kind']} {r['lockin_data2']} {r['lockin_data2_unit']}")
            elif r.get("focus_position"):
                note = f"lens {r['focus_position']}"
                if r.get("focus_converged") == "False":
                    note += " (not converged)"
            elif r["error"]:
                note = f"⚠️ {r['error']}"
            state = {"ok": "✅", "aborted": "⛔"}.get(r["status"], "❌")
            lines.append(f"| {r['step_index']} | {r['elapsed_s']:.1f} | {r['action']} "
                         f"| {r['robot_id']} | {r['iteration']} | {state} "
                         f"| {r['duration_s']} | {note} |")
        lines.append("")
        lines.append(f"---\n生成: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} / "
                     f"元JSON: `{os.path.abspath(self.source_json_path)}`")

        with open(self.summary_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
