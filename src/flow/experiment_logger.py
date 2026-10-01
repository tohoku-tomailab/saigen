"""
L2層: 実験ログ管理クラス（ExperimentLogger）

実験（ワークフロー）1回の実行ごとに専用フォルダを作成し、
トレーサビリティ（再現性確保）のために以下を残す:

    logs/<日付>/<実験名>_<タイムスタンプ>/
        run.log          全ログ（コンソール出力と同じ内容をファイルにも保存）
        <入力ファイル>   実行に使った入力ファイルのコピー（JSON/CSV、元のファイル名を保持）
        metadata.json    実行メタデータ（開始/終了時刻・使用ロボット・結果ステータス等）
        measurements.csv 各ステップの構造化記録（重量・画像パス・成否・所要時間）
        summary.md       天秤の測定値と画像を時系列でひも付けた人間可読レポート
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
import os
import re
import shutil
import time
from datetime import datetime


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
                    status, duration, result=None, error=None):
        """1ステップの実行結果を記録

        Args:
            index: ステップ番号（1始まり、ループ展開後）
            total: 総ステップ数
            action: アクション名
            robot_id: ロボットID（共有デバイスは None）
            iteration: ループ反復番号（ループ外は None）
            status: "ok" or "error"
            duration: 所要秒数
            result: execute_step の戻り値（{"weight":..} / {"image_path":..} 等）
            error: 例外メッセージ（失敗時）
        """
        now = datetime.now()
        elapsed = time.monotonic() - self._t0
        result = result or {}
        weight = result.get("weight")
        image_path = result.get("image_path")
        nominal_time = result.get("nominal_time_s")
        focus_position = result.get("focus_position")
        focus_converged = result.get("focus_converged")
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
            "error": error or "",
        }
        self.records.append(record)

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
                  "focus_converged", "error"]
        with open(self.csv_path, "w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in self.records:
                writer.writerow(r)

    def _write_metadata(self):
        weights = [r for r in self.records if r["weight_g"] != ""]
        images = [r for r in self.records if r["image_path"] != ""]
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
            "robots_used": sorted(self.robots_used),
            "picus2_robots": sorted(self.picus2_robots),
            # JSON のキーは文字列になる: {"4": "maxarm"}
            "robot_types": {str(rid): t for rid, t in sorted(self.robot_types.items())},
            "weight_measurements": len(weights),
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
                     f" 失敗 {sum(1 for r in self.records if r['status']=='error')}）")
        if self.robots_used:
            lines.append(f"- **使用ロボット**: {sorted(self.robots_used)}")
            if self.robot_types:
                lines.append("- **ロボット種別**: " + ", ".join(
                    f"Robot {rid} = {t}" for rid, t in sorted(self.robot_types.items())))
        lines.append(f"- **重量測定**: {len(weights)} 回 / **撮影画像**: {len(images)} 枚")
        if self.error:
            lines.append(f"- **エラー**: {self.error}")
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
                kind = "⚖️ 重量" if e["kind"] == "weight" else "📷 画像"
                detail = e["detail"]
                if e["kind"] == "image":
                    rel = os.path.relpath(detail, self.dir) if os.path.isabs(detail) else detail
                    detail = f"[{os.path.basename(detail)}]({rel.replace(os.sep, '/')})"
                lines.append(f"| {e['elapsed_s']:.1f} | {e['timestamp'].split(' ')[1]} "
                             f"| {kind} | {e['step_index']} | {e['iteration']} | {detail} |")
        else:
            lines.append("（重量測定・画像撮影はありませんでした）")
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
            elif r.get("focus_position"):
                note = f"lens {r['focus_position']}"
                if r.get("focus_converged") == "False":
                    note += " (not converged)"
            elif r["error"]:
                note = f"⚠️ {r['error']}"
            state = "✅" if r["status"] == "ok" else "❌"
            lines.append(f"| {r['step_index']} | {r['elapsed_s']:.1f} | {r['action']} "
                         f"| {r['robot_id']} | {r['iteration']} | {state} "
                         f"| {r['duration_s']} | {note} |")
        lines.append("")
        lines.append(f"---\n生成: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} / "
                     f"元JSON: `{os.path.abspath(self.source_json_path)}`")

        with open(self.summary_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
