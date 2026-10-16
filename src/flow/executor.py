import asyncio
import logging
import sys
import os

# パス解決: リポジトリルートを sys.path に追加（直接実行時に案内メッセージを出すため）
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


# ロギング設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
_default_logger = logging.getLogger(__name__)

# 共有デバイス用アクション（robot_idに依存しない）
SHARED_DEVICE_ACTIONS = {"measure_weight", "tare_scale", "capture_and_save",
                         "capture_microscope", "microscope_led", "microscope_focus",
                         "measure_dmm", "measure_lockin"}
MICROSCOPE_ACTIONS = {"capture_microscope", "microscope_led", "microscope_focus"}
# 顕微鏡のカメラ（UVC）が要るアクションと、制御ポート（シリアル）が要るアクション
MICROSCOPE_CAMERA_ACTIONS = {"capture_microscope", "microscope_focus"}
MICROSCOPE_SERIAL_ACTIONS = {"microscope_led", "microscope_focus"}
# Keithley 2002 DMM（GPIB）が要るアクション
DMM_ACTIONS = {"measure_dmm"}
# NF LI5640 ロックインアンプ（GPIB）が要るアクション
LOCKIN_ACTIONS = {"measure_lockin"}

# ループ制御アクション（実行時にスキップ）
LOOP_CONTROL_ACTIONS = {"loop_start", "loop_end"}


def expand_loops(steps: list) -> list:
    """
    loop_start/loop_endマーカーを展開してフラットなリストに変換

    Args:
        steps: ステップリスト（loop_start/loop_endを含む可能性あり）

    Returns:
        展開後のフラットなステップリスト

    Raises:
        ValueError: ループ構造が不正な場合
    """
    expanded = []
    i = 0

    while i < len(steps):
        step = steps[i]
        action = step.get("action") if isinstance(step, dict) else step.action

        if action == "loop_start":
            loop_id = step.get("loop_id") if isinstance(step, dict) else step.loop_id
            count = step.get("count") if isinstance(step, dict) else step.count

            # 対応するloop_endを探す
            loop_body = []
            j = i + 1
            found_end = False

            while j < len(steps):
                inner = steps[j]
                inner_action = inner.get("action") if isinstance(inner, dict) else inner.action
                inner_id = inner.get("loop_id") if isinstance(inner, dict) else getattr(inner, "loop_id", None)

                if inner_action == "loop_end" and inner_id == loop_id:
                    found_end = True
                    break
                elif inner_action == "loop_end":
                    # 別 id の loop_end をループ本体に含めると、構造の誤りが
                    # 黙って「普通のステップ」として展開されてしまう
                    raise ValueError(
                        f"loop_start (id={loop_id}) の本体に対応しない loop_end "
                        f"(id={inner_id}) があります（ステップ {j + 1}）"
                    )
                elif inner_action == "loop_start":
                    # ネストループは現在非対応
                    raise ValueError(f"ネストされたループは現在サポートされていません（loop_id: {loop_id}）")
                else:
                    loop_body.append(inner)
                j += 1

            if not found_end:
                raise ValueError(f"loop_start (id={loop_id}) に対応する loop_end が見つかりません")

            # count回展開
            for iteration in range(count):
                for body_step in loop_body:
                    if isinstance(body_step, dict):
                        copied = body_step.copy()
                        copied["_iteration"] = iteration + 1
                    else:
                        copied = body_step
                    expanded.append(copied)

            i = j + 1  # loop_endの次へ

        elif action == "loop_end":
            # 対応するloop_startなしのloop_end
            loop_id = step.get("loop_id") if isinstance(step, dict) else step.loop_id
            raise ValueError(f"loop_end (id={loop_id}) に対応する loop_start が見つかりません")

        else:
            expanded.append(step)
            i += 1

    return expanded


async def execute_shared_device_step(step, shared_devices, logger=None):
    """
    共有デバイス用のステップを実行

    Args:
        step: アクション情報（Pydanticモデル or 辞書）
        shared_devices: SharedDevicesインスタンス
        logger: ロガー（省略時はデフォルト）
    """
    if logger is None:
        logger = _default_logger

    # 辞書とPydanticモデルの両方に対応
    action = step.get("action") if isinstance(step, dict) else step.action

    if shared_devices is None:
        raise RuntimeError("SharedDevicesが初期化されていません")

    # ===== カメラ操作 =====
    if action == "capture_and_save":
        file_path = step.get("file_path") if isinstance(step, dict) else step.file_path
        # file_pathが空文字列の場合はNoneに変換（自動生成）
        if not file_path:
            file_path = None
        saved_path = await shared_devices.capture_and_save(file_path)
        if not saved_path:
            # SharedDevices は失敗時に None を返す。成功扱いにすると画像の無い
            # 「ok」ステップが記録に残るので、ステップの失敗として止める
            raise RuntimeError("カメラ撮影に失敗しました（画像が保存されませんでした）")
        return {"image_path": saved_path}

    # ===== デジタル顕微鏡操作 =====
    elif action == "capture_microscope":
        file_path = step.get("file_path") if isinstance(step, dict) else step.file_path
        if not file_path:
            file_path = None
        saved_path = await shared_devices.capture_microscope(file_path)
        if not saved_path:
            raise RuntimeError("顕微鏡撮影に失敗しました（画像が保存されませんでした）")
        return {"image_path": saved_path}

    elif action == "microscope_led":
        on = step.get("on", True) if isinstance(step, dict) else step.on
        level = step.get("level") if isinstance(step, dict) else step.level
        state = await shared_devices.set_microscope_led(on, level)
        logger.info(f"  顕微鏡 LED: {'ON' if state else 'OFF'}")
        return None

    elif action == "microscope_focus":
        if isinstance(step, dict):
            kwargs = {k: step.get(k) for k in ("mode", "position", "direction", "steps", "timeout")
                      if step.get(k) is not None}
        else:
            kwargs = {"mode": step.mode, "position": step.position, "direction": step.direction,
                      "steps": step.steps, "timeout": step.timeout}
        result = await shared_devices.focus_microscope(**kwargs)
        logger.info(f"  顕微鏡フォーカス: 位置 {result.get('focus_position')} "
                    f"({'収束' if result.get('focus_converged') else '未収束'})")
        return result

    # ===== 電子天秤操作（BCE8221） =====
    elif action == "measure_weight":
        stabilization_count = step.get("stabilization_count", 3) if isinstance(step, dict) else step.stabilization_count
        weight = await shared_devices.measure_weight(stabilization_count)
        logger.info(f"  測定結果: {weight:.3f}g")
        return {"weight": weight}

    elif action == "tare_scale":
        delay = step.get("delay", 1.0) if isinstance(step, dict) else step.delay
        await shared_devices.tare_scale(delay)
        return None

    # ===== DMM 操作（Keithley 2002） =====
    elif action == "measure_dmm":
        if isinstance(step, dict):
            kwargs = {"function": step.get("function", "dc_voltage"),
                      "samples": step.get("samples", 3),
                      "nplc": step.get("nplc", 1.0),
                      "range_": step.get("range")}
        else:
            kwargs = {"function": step.function, "samples": step.samples,
                      "nplc": step.nplc, "range_": step.range}
        result = await shared_devices.measure_dmm(**kwargs)
        logger.info(f"  DMM: {result['dmm_value']!r} {result['dmm_unit']} ({result['dmm_function']})")
        return result

    # ===== ロックインアンプ（NF LI5640、読み取り専用） =====
    elif action == "measure_lockin":
        if isinstance(step, dict):
            kwargs = {"samples": step.get("samples", 5), "interval": step.get("interval", 0.0)}
        else:
            kwargs = {"samples": step.samples, "interval": step.interval}
        result = await shared_devices.measure_lockin(**kwargs)
        logger.info(f"  LI5640: {result['lockin_data1_kind']} {result['lockin_data1']!r} "
                    f"{result['lockin_data1_unit']}, {result['lockin_data2_kind']} "
                    f"{result['lockin_data2']!r} {result['lockin_data2_unit']}")
        return result

    else:
        # 未知のアクションを警告だけで読み飛ばすと、後続ステップが想定外の
        # 状態で実行される。実行を止める。
        raise ValueError(f"不明な共有デバイスアクション: {action!r}")


async def execute_robot_step(step, robots, logger=None):
    """
    ロボットアーム用のステップを実行

    Args:
        step: アクション情報（Pydanticモデル or 辞書）
        robots: {1: LabRobot, 2: LabRobot} または単一のLabRobotインスタンス（後方互換性）
        logger: ロガー（省略時はデフォルト）
    """
    if logger is None:
        logger = _default_logger

    # 辞書とPydanticモデルの両方に対応
    action = step.get("action") if isinstance(step, dict) else step.action

    # robot_idでルーティング（デフォルト: 1）
    robot_id = step.get("robot_id", 1) if isinstance(step, dict) else getattr(step, "robot_id", 1)

    # 後方互換性: 単一robotの場合はそのまま使用
    if isinstance(robots, dict):
        robot = robots.get(robot_id)
        if robot is None:
            raise RuntimeError(f"Robot {robot_id}が初期化されていません")
    else:
        robot = robots

    # ロボット種別ごとの対応アクション（MaxArm は SUPPORTED_ACTIONS を持つ）。
    # プリフライト（run_flow.preflight_capabilities）でも検査するが、ライブラリ
    # 経路でも未対応のアクションをドライバへ送らないよう、ここでも止める。
    supported = getattr(robot, "SUPPORTED_ACTIONS", None)
    if supported is not None and action not in supported:
        rtype = getattr(robot, "ROBOT_TYPE", type(robot).__name__)
        raise RuntimeError(
            f"Robot {robot_id}（{rtype}）はアクション {action!r} に対応していません"
            f"（対応: {', '.join(sorted(supported))}）"
        )

    if action == "move_xyz":
        x = step.get("x") if isinstance(step, dict) else step.x
        y = step.get("y") if isinstance(step, dict) else step.y
        z = step.get("z") if isinstance(step, dict) else step.z
        await robot.move_xyz(x, y, z)

    elif action == "move_z":
        distance = step.get("distance") if isinstance(step, dict) else step.distance
        await robot.move_z(distance)

    elif action == "move_radial":
        distance = step.get("distance") if isinstance(step, dict) else step.distance
        await robot.move_radial(distance)

    elif action == "rotate":
        angle = step.get("angle") if isinstance(step, dict) else step.angle
        await robot.rotate(angle)

    elif action == "rotate_relative":
        angle = step.get("angle") if isinstance(step, dict) else step.angle
        speed = step.get("speed") if isinstance(step, dict) else getattr(step, "speed", None)
        await robot.rotate_relative(angle, speed=speed)

    elif action == "go_home":
        await robot.go_home()

    elif action == "wait":
        seconds = step.get("seconds", 1) if isinstance(step, dict) else step.seconds
        logger.info(f"  {seconds}秒 待機中...")
        await asyncio.sleep(seconds)

    # ===== Picus2（電動ピペット）操作 =====
    elif action == "aspirate":
        volume = step.get("volume") if isinstance(step, dict) else step.volume
        speed = step.get("speed", 5) if isinstance(step, dict) else step.speed
        # 公称所要時間などの記録用データを伝播する
        return await robot.aspirate(volume, speed)

    elif action == "dispense":
        volume = step.get("volume") if isinstance(step, dict) else step.volume
        speed = step.get("speed", 5) if isinstance(step, dict) else step.speed
        return await robot.dispense(volume, speed)

    elif action == "blow_out":
        go_home = step.get("go_home", True) if isinstance(step, dict) else step.go_home
        speed = step.get("speed", 1) if isinstance(step, dict) else step.speed
        delay_ms = step.get("delay_ms", 3000) if isinstance(step, dict) else step.delay_ms
        await robot.blow_out(go_home, speed, delay_ms)

    # ===== MaxArm（吸着ノズル）操作 =====
    elif action == "suction":
        on = step.get("on", True) if isinstance(step, dict) else step.on
        await robot.suction(bool(on))

    # ===== スライダー・コンベア操作 =====
    elif action == "move_slider":
        position = step.get("position") if isinstance(step, dict) else step.position
        await robot.move_slider(position)

    elif action == "move_conveyer":
        index = step.get("index", 0) if isinstance(step, dict) else step.index
        speed = step.get("speed") if isinstance(step, dict) else step.speed
        duration = step.get("duration") if isinstance(step, dict) else step.duration
        await robot.move_conveyer(index, speed, duration)

    else:
        raise ValueError(f"不明なアクション: {action!r}")


async def execute_step(step, robots, shared_devices=None, logger=None):
    """
    1つのステップを実行（共通処理）

    Args:
        step: アクション情報（Pydanticモデル or 辞書）
        robots: {1: LabRobot, 2: LabRobot} または単一のLabRobotインスタンス（後方互換性）
        shared_devices: SharedDevicesインスタンス（共有デバイス操作時に必要）
        logger: ロガー（省略時はデフォルト）

    新しいアクションを追加する場合:
    - ロボットアーム操作: execute_robot_step に elif を追加
    - 共有デバイス操作: execute_shared_device_step に elif を追加し、
      SHARED_DEVICE_ACTIONS にアクション名を追加
    """
    if logger is None:
        logger = _default_logger

    # 辞書とPydanticモデルの両方に対応
    action = step.get("action") if isinstance(step, dict) else step.action

    # 共有デバイスアクションかロボットアクションかを判定
    # 戻り値: 測定結果などの記録用データ（{"weight":..} / {"image_path":..}）または None を伝播する
    if action in SHARED_DEVICE_ACTIONS:
        return await execute_shared_device_step(step, shared_devices, logger)
    else:
        return await execute_robot_step(step, robots, logger)


async def execute_workflow(json_path: str, robot_settings: dict = None, shared_settings: dict = None):
    """廃止: 使わないこと。

    旧実装は ExperimentSession の安全枠を通らずに実機を開き、ループを展開せず、
    全ステップを robot_id に関係なく 1 台のアームへ送っていた。Mock の選択肢も
    無かった。フローの実行は run_flow を使う::

        python -m src.flow.run_flow <flow.json> --validate-only
        python -m src.flow.run_flow <flow.json> --mock

    Raises:
        NotImplementedError: 常に
    """
    raise NotImplementedError(
        "execute_workflow() は廃止されました。"
        "`python -m src.flow.run_flow <flow.json> --mock`（実機は --mock なし）を使ってください。"
    )


if __name__ == "__main__":
    # 以前はここで ZIF-8 の例を実機（COM3）で実行していた。誤起動で
    # アームが動かないよう、案内だけ表示して終了する。
    print(
        "src/flow/executor.py は直接実行できません。フローは次のように実行します:\n"
        "  python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --validate-only\n"
        "  python -m src.flow.run_flow examples/zif8/zif8_two_solution_mixing_speed5.json --mock",
        file=sys.stderr,
    )
    sys.exit(2)
