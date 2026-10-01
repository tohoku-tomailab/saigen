"""
LLM Service - 自然言語からのワークフロー（実験フロー）生成

LLM_MODE 環境変数でバックエンドを切替:
  - gemini (デフォルト): Google Gemini API（GEMINI_API_KEY が必要）
  - openai_compatible : 任意の OpenAI 互換エンドポイント
                        （LLM_BASE_URL / LLM_API_KEY / LLM_MODEL で指定。
                          ローカル推論サーバや他社 API に向けられる）

いずれのキーもコードに埋め込まず、.env（python-dotenv）から読み込む。
"""
import os
import re
import json
from dotenv import load_dotenv

load_dotenv()

try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False

try:
    import openai  # noqa: F401
    OPENAI_SDK_AVAILABLE = True
except ImportError:
    OPENAI_SDK_AVAILABLE = False

GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-3.5-flash")


def _llm_mode() -> str:
    return os.getenv("LLM_MODE", "gemini").strip().lower()


def _is_configured() -> bool:
    """選択中のバックエンドが利用可能か（依存とキーの有無）を返す"""
    mode = _llm_mode()
    if mode == "openai_compatible":
        return OPENAI_SDK_AVAILABLE and bool(os.getenv("LLM_BASE_URL"))
    return GEMINI_AVAILABLE and bool(os.getenv("GEMINI_API_KEY"))


LLM_AVAILABLE = _is_configured()


SYSTEM_PROMPT = """
あなたはラボ用ロボットアームのワークフローを生成するアシスタントです。
ユーザーの自然言語の指示を、以下のJSON形式に変換してください。

## 出力形式
{"name": "名前", "description": "説明", "steps": [...]}

## ロボットID（robot_id）について
- システムには3台のDobotロボットアームと、任意で1台のMaxArmがあります
  - Robot 1 (robot_id: 1): メインロボット。コンベアベルトもRobot 1が制御
  - Robot 2 (robot_id: 2): サブロボット
  - Robot 3 (robot_id: 3): スライダーレール上のロボット。ピペットなし
  - Robot 4 (robot_id: 4): 任意の Hiwonder MaxArm（吸着ノズルによるピック＆プレース用）。ピペットなし。
    使えるアクションは move_xyz / move_z / go_home / suction / wait だけ
    （rotate_relative, move_radial, aspirate, dispense, blow_out, move_slider, move_conveyer は使えない）。
    座標は MaxArm 自身の座標系（mm、原点は台座の回転軸、アームはホームで -Y 方向を向く）で、
    Dobot の座標とは別物。ユーザーが MaxArm / 吸着 / ピック＆プレースを指示したときだけ使う
- 各ステップに "robot_id": 1, 2, 3, または 4 を指定します
- 指定がない場合やデフォルトは robot_id: 1 です

## 利用可能なアクション

### ロボットアーム操作（Dobot）
- move_xyz: {"action": "move_xyz", "robot_id": 1, "x": 200.0, "y": 100.0, "z": 50.0}
- move_z: {"action": "move_z", "robot_id": 1, "distance": 10.0} (正で上昇、負で下降)
- move_radial: {"action": "move_radial", "robot_id": 1, "distance": 20.0} (半径方向の相対移動。正=外向き/基部から離れる、負=内向き/基部に近づく。アームの向きは維持)
- rotate_relative: {"action": "rotate_relative", "robot_id": 1, "angle": 90.0, "speed": "low"} (現在位置から相対回転。正=反時計回り（上から見て、+Y方向）、負=時計回り。ロボットの正面（+X 側）に立ってロボットに向かい合う操作者（operator facing the robot）から見ると、正=アーム先端が操作者の右へ、負=左へ動く。指示の「右」「左」はこの操作者の視点と解釈する。speed: "low"=低速, "normal"=普通, "high"=高速。デフォルト: "low")
- go_home: {"action": "go_home", "robot_id": 1}
- move_slider: {"action": "move_slider", "robot_id": 1, "position": 500.0} (スライダーを絶対位置に移動。0-1000mm)
- move_conveyer: {"action": "move_conveyer", "robot_id": 1, "index": 0, "speed": 10.0, "duration": 5.0} (コンベアベルトを動作。speed: 速度mm/s, duration: 動作時間秒)

### 電動ピペット操作（Picus2）
- aspirate: {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 5} (液体吸引。volume: 0.5-10.0 mL（実機ピペットの最小操作量 0.5 mL）, speed: 1-9)
- dispense: {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 5} (液体分注。volume: 0.5-10.0 mL（実機ピペットの最小操作量 0.5 mL）, speed: 1-9)
- blow_out: {"action": "blow_out", "robot_id": 1, "go_home": true, "speed": 1, "delay_ms": 3000} (残液完全排出)

### 吸着ノズル操作（MaxArm, Robot 4 専用）
- suction: {"action": "suction", "robot_id": 4, "on": true} (on: true=吸着（ポンプ ON）, false=解放。Dobot（robot_id 1-3）には使えない)

### カメラ操作（Webcam）※共有デバイスのためrobot_id不要
- capture_and_save: {"action": "capture_and_save", "file_path": "captured_images/image.jpg"}

### デジタル顕微鏡操作（USB 顕微鏡, 例: 400-CAM106）※共有デバイスのためrobot_id不要
- capture_microscope: {"action": "capture_microscope", "file_path": ""} (顕微鏡で撮影。file_path 空欄で自動生成)
- microscope_led: {"action": "microscope_led", "on": true, "level": 12} (顕微鏡の LED 照明。on: true=点灯/false=消灯, level: 明るさ 0-255 省略可)
- microscope_focus: {"action": "microscope_focus", "mode": "auto", "timeout": 60.0} (顕微鏡の焦点合わせ。mode: "auto"=ワンショットAF / "position"=レンズ位置指定 (position: 0-65535) / "step"=ステップ移動 (direction: "in"/"out", steps))

### 電子天秤操作（BCE8221）※共有デバイスのためrobot_id不要
- measure_weight: {"action": "measure_weight", "stabilization_count": 3} (重量測定)
- tare_scale: {"action": "tare_scale", "delay": 1.0} (風袋引き)

### ユーティリティ
- wait: {"action": "wait", "robot_id": 1, "seconds": 2.0}

### ループ制御
- loop_start: {"action": "loop_start", "loop_id": "loop1", "count": 3}
- loop_end: {"action": "loop_end", "loop_id": "loop1"}

## 正解例

### 例1: 「50mm下降して、5ml吸引して50mm上昇」
```json
{
  "name": "吸引操作",
  "description": "50mm下降して5ml吸引し、50mm上昇する",
  "steps": [
    {"action": "move_z", "robot_id": 1, "distance": -50.0},
    {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 5},
    {"action": "move_z", "robot_id": 1, "distance": 50.0},
    {"action": "go_home", "robot_id": 1}
  ]
}
```

### 例2: 「（ロボットに向かい合う操作者から見て）右に90°回転して70mm下降して5ml排出して70mm上昇してホームに戻る」
```json
{
  "name": "分注後帰還",
  "description": "右に90度回転し、70mm下降して5ml排出後、上昇してホームへ戻る",
  "steps": [
    {"action": "rotate_relative", "robot_id": 1, "angle": 90.0, "speed": "low"},
    {"action": "move_z", "robot_id": 1, "distance": -70.0},
    {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 5},
    {"action": "move_z", "robot_id": 1, "distance": 70.0},
    {"action": "go_home", "robot_id": 1}
  ]
}
```

### 例3: 分注サイクル（吸引→移動→排出→帰還）
指示: 「ビーカーAから5ml吸引してビーカーBに排出する。ビーカーAは開始地点の50mm下、ビーカーBは開始地点右90°50mm下の位置にある」
```json
{
  "name": "分注サイクル",
  "description": "ビーカーAからビーカーBへの5ml移送",
  "steps": [
    {"action": "move_z", "robot_id": 1, "distance": -50.0},
    {"action": "aspirate", "robot_id": 1, "volume": 5.0, "speed": 5},
    {"action": "move_z", "robot_id": 1, "distance": 50.0},
    {"action": "rotate_relative", "robot_id": 1, "angle": 90.0, "speed": "low"},
    {"action": "move_z", "robot_id": 1, "distance": -50.0},
    {"action": "dispense", "robot_id": 1, "volume": 5.0, "speed": 5},
    {"action": "move_z", "robot_id": 1, "distance": 50.0},
    {"action": "go_home", "robot_id": 1}
  ]
}
```
##ルール
- ループ動作の一番最後に、ホームポジションに戻る動作を必ず入れること。（繰り返しごとにその都度ホームポジションに戻る。）
- 一番最後のステップは必ずホームポジションに戻る動作(go_home)で終わるようにしてください。

## 重要な出力ルール
- 最終的なJSONのみを1つだけ出力してください
"""

def get_gemini_client():
    """Geminiクライアントを初期化"""
    if not GEMINI_AVAILABLE:
        raise ImportError("google-generativeai がインストールされていません")

    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY が設定されていません")

    genai.configure(api_key=api_key)
    return genai.GenerativeModel(GEMINI_MODEL)


def _build_prompt(user_input: str, template: dict = None) -> str:
    """SYSTEM_PROMPTとユーザー入力からプロンプトを組み立てる"""
    if template:
        return (
            f"{SYSTEM_PROMPT}\n\n"
            f"以下のJSONテンプレートのパラメータを、ユーザーの指示に従って変更してください。\n"
            f"テンプレートの構造（ステップの数・順序・種類、robot_id、loop_id）は変更せず、指定されたパラメータの値のみ変更してください。構造が変わった出力は採用されません。\n\n"
            f"テンプレートJSON:\n{json.dumps(template, ensure_ascii=False)}\n\n"
            f"ユーザーの指示:\n{user_input}"
        )
    return f"{SYSTEM_PROMPT}\n\n指示: {user_input}"


class TemplateStructureError(ValueError):
    """テンプレート指定の生成で、LLM がステップの構造を変えた。

    テンプレートは実機で確認済みの手順なので、変えてよいのはパラメータ
    （移動量・角度・容量・速度・待ち時間・ループ回数など）だけ。ステップの
    追加・削除・並べ替え・アクションや robot_id / loop_id の変更は拒否する。
    """


# テンプレートの「構造」とみなすキー（値が変わったら構造変更）
_STRUCTURAL_KEYS = ("action", "robot_id", "loop_id")


def _step_signature(step) -> tuple:
    if not isinstance(step, dict):
        return ("<not an object>",)
    return tuple(step.get(k) for k in _STRUCTURAL_KEYS)


def check_template_structure(template: dict, generated) -> None:
    """生成結果がテンプレートと同じ構造（同じアクションを同じ順序で、同じ
    robot_id / loop_id）かを確認し、違えば :class:`TemplateStructureError`。

    パラメータの値、および省略可能なパラメータの追加・省略は許す（それは
    スキーマ検証 src/flow/schema.py が別途確認する）。
    """
    if not isinstance(generated, dict) or not isinstance(generated.get("steps"), list):
        raise TemplateStructureError("生成結果に steps の一覧がありません（テンプレートの構造と異なります）")
    t_steps = template.get("steps") or []
    g_steps = generated["steps"]
    problems = []
    if len(t_steps) != len(g_steps):
        problems.append(f"ステップ数が変わっています（テンプレート {len(t_steps)} → 生成 {len(g_steps)}）")
    for i, (t, g) in enumerate(zip(t_steps, g_steps), 1):
        ts, gs = _step_signature(t), _step_signature(g)
        if ts != gs:
            diffs = [f"{k}: {a!r} → {b!r}" for k, a, b in zip(_STRUCTURAL_KEYS, ts, gs) if a != b]
            problems.append(f"ステップ {i}: " + ", ".join(diffs or ["形式が異なります"]))
        if len(problems) >= 10:
            problems.append("...")
            break
    if problems:
        raise TemplateStructureError(
            "生成結果がテンプレートの構造を変えたため採用しません"
            "（テンプレートで変えられるのはパラメータの値だけです）:\n- " + "\n- ".join(problems)
        )


def _generate_openai_compatible(prompt: str) -> str:
    """OpenAI 互換エンドポイント（ローカル推論サーバ等）でテキスト生成

    接続先は LLM_BASE_URL、認証は LLM_API_KEY、モデル名は LLM_MODEL で指定する。
    """
    # 遅延import: LLM_MODE=gemini 運用時に不要な依存をロードしない
    from openai import OpenAI

    base_url = os.getenv("LLM_BASE_URL")
    if not base_url:
        raise ValueError("LLM_MODE=openai_compatible では LLM_BASE_URL の設定が必要です")

    client = OpenAI(
        base_url=base_url,
        api_key=os.getenv("LLM_API_KEY", "not-needed"),
    )
    try:
        resp = client.chat.completions.create(
            model=os.getenv("LLM_MODEL", "default"),
            messages=[{"role": "user", "content": prompt}],
            temperature=0,
            max_tokens=2048,
        )
    except Exception as e:
        raise RuntimeError(
            f"LLM エンドポイント ({base_url}) に接続できません: {e}\n"
            f"サーバの起動状態と LLM_BASE_URL を確認するか、LLM_MODE=gemini に切り替えてください"
        )
    return resp.choices[0].message.content


def _generate_gemini(prompt: str) -> str:
    """Gemini API でテキスト生成（デフォルト経路）"""
    model = get_gemini_client()
    response = model.generate_content(
        prompt,
        generation_config=genai.GenerationConfig(
            temperature=0,
            response_mime_type="application/json"
        )
    )
    return response.text


def _generate(prompt: str, mode: str) -> str:
    """LLM_MODE に応じてバックエンドを選択"""
    if mode == "openai_compatible":
        return _generate_openai_compatible(prompt)
    if mode == "gemini":
        return _generate_gemini(prompt)
    raise ValueError(f"不明な LLM_MODE: {mode}（gemini / openai_compatible）")


def _strip_json_fences(raw: str) -> str:
    """```json フェンスや前後の説明文を取り除き、JSON本体を抽出する"""
    text = (raw or "").strip()
    match = re.search(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL)
    if match:
        return match.group(1)
    return text


def _parse_json_with_retry(raw: str, prompt: str, mode: str) -> dict:
    """JSONパース。失敗時は1回だけリトライ（無限ループ防止のため最大1回）"""
    try:
        return json.loads(_strip_json_fences(raw))
    except json.JSONDecodeError:
        pass

    retry_prompt = (
        f"{prompt}\n\n"
        f"前回の出力はJSONパースに失敗しました。説明文やコードフェンスを付けず、"
        f"純粋なJSONのみを出力してください。"
    )
    raw = _generate(retry_prompt, mode)
    try:
        return json.loads(_strip_json_fences(raw))
    except json.JSONDecodeError as e:
        raise ValueError(f"LLMが不正なJSONを返しました: {e}")


def generate_workflow_from_prompt(user_input: str, template: dict = None) -> dict:
    """自然言語の入力からワークフローJSONを生成

    Args:
        user_input: ユーザーの自然言語指示
        template: プリセットのテンプレートJSON（指定時はパラメータ変更のみ行う）

    Raises:
        TemplateStructureError: ``template`` 指定時に、生成結果のステップ構造
            （アクションの種類と順序、robot_id、loop_id）がテンプレートと違う
        ValueError / RuntimeError: 生成・JSON パースに失敗した。失敗時に
            テンプレートをそのまま返すことはしない（呼び出し側が失敗を表示する）
    """
    prompt = _build_prompt(user_input, template)
    mode = _llm_mode()
    try:
        raw = _generate(prompt, mode)
        result = _parse_json_with_retry(raw, prompt, mode)
    except (ValueError, RuntimeError):
        raise
    except Exception as e:
        raise RuntimeError(f"ワークフロー生成に失敗しました: {e}")
    if template:
        check_template_structure(template, result)
    return result
