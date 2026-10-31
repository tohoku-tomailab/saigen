# SAIGEN 改善計画（2026-10-30、改訂 2026-10-31）

対象: `f1665b4`（HEAD `102e617` との差は本書だけ）。根拠は Codex（gpt-6-astra）による 3 回の評価（2026-10-21 プロジェクト全体、2026-10-30 「Python とフロー設計の分離」、2026-10-30 本計画書 v1 の反証レビュー）と、その指摘を Claude がコードで裏取りした結果。評価の出力はセッションの一時領域にしかないので、根拠は本書に file:line で写してある。

この文書は「何を・どの順で・どこまで」を決める計画である。各項目の実装手順書（タスク分割・テスト・コミット単位）は、着手するときに `docs/superpowers/plans/` に別途書く。

## 改訂履歴

- v1（2026-10-30、`102e617`）: 初版。
- v2（2026-10-31）: v1 への反証レビューで指摘された 5 点を反映。(1) 1a の安全契約を具体化、(2) 1b/1c を session と送信境界まで拡張し 1e/1f を追加、(3) Phase 0 の成立しない検査（半径反転の反例、文字列 grep、CI の依存セット、非スキップ確認、「全 examples 成功」）を修正、(4) 2a〜2c の前に共通契約 C1〜C5 を置く、(5) 3a〜3i の保証範囲を修正（25 種類、動作表と装置表の分離、整合テストの限界、生成物の安全層編集、ZIF-8 の前提）。加えて行番号 4 件、状態名 `stopped`、「安全層に触れない」の矛盾、規模見積もりを直した。
- v2 の再評価（2026-10-31）: 5 件のうち「3a〜3i の保証範囲」は反映済み、他の 4 件は一部反映と判定された。**安全層の実装にはまだ着手しない。** 次版で直す点は 5 件。(1) 停止の確認と送信を同じ排他規則で直列化し、停止後に始まった新しい操作を拒否する。(2) 検証した具体的な指令をそのまま送る。ドライバが姿勢を読み直して目標を再計算する経路（`move_z` など）を含める。(3) C1・C2・1c・1e を一つの異常終了契約にする。既存の `ValueError` 拒否と、検証前の速度設定の書き込みを含め、初期化中の中断も扱う。(4) 0-1 の反例に開始姿勢と 4 要素のホームを明記する。(e)(f) の一律 xfail はやめる。(5) Dobot へ転送する設定を許可リストにする（`dobot_homing` を通さない）。1f の依存を C2・C5 に結ぶ。安全層に触れない 0-4、0-8、0-2 は先行してよい。

## 方針

1. **安全 → 再現性 → 分離（保守性）の順。** ただし全項目をこの順に直列化はしない。反例テストと、安全層に触れない文書・CI の是正は並行して進める。
2. **安全層の変更は人の決定が要る。** `src/devices/safety/` を編集する項目は本書で明示する（0-6、1a、1b、1c、1e、1f、3h）。反例テストを先に置き、人が「直す」と決めてから実装する。AI は限界を緩めない。
3. **反例を先にテストにする。** 実機ラッパーと mock は同じ構造を共有しているので、mock との一致テストでは検出できない。制約そのものに対する反例を、両方に独立に置く。反例は「拒否された」だけでなく「送信された指令列」で確かめる。
4. **拒否と故障を例外の型で分ける。** 「送信前に検証で拒否した」ことと「送信後に装置が失敗した」ことは復帰の扱いが違う。前者で復帰動作を走らせてはならない（C1）。
5. **予測は実測の代わりにならない。** preflight の姿勢推定は補助であり、実行時の実測姿勢に対する検査を省く根拠にしない。
6. **実機での確認は人の作業。** 回転の経路や停止の挙動はコードから導いた推論であって実測ではない。Phase 1 の受入には bench での確認が要る。

## 決定済み（2026-10-30、AskUserQuestion で確認。v2 で D1 を具体化）

| # | 決定 | 採用 | 理由・v2 での補足 |
|---|---|---|---|
| D1 | Phase 1a の検査範囲 | **終点＋回転経路** | v1 は「5° 刻みの点検査」としたが、固定刻みは保証にならない（半径 200 mm、−2.5°→+2.5°、X 上限 199.9 mm で両端は X≈199.81 で通り、0° で X=200 が違反）。v2: **端点に加えて、回転区間内で X・Y が極値をとる角度（90° の倍数のうち区間に入るもの）を必ず検査**する。刻み点は補助。「刻みが細かいから安全」は受入根拠にしない |
| D2 | Phase 1c: エラー後の自動ホーム復帰 | **既定で無効**（`recover_home_after_error: true` で opt-in） | 液体・治具を保持した状態で動いてよいかは研究室ごとに違う。v2: 復帰しない場合は「停止要求 → 切断」であり、ホームを省くだけではない。session 側の復帰経路も対象 |
| D3 | Phase 1d: センサーの `is_safe` | **まず位置づけ直し**。インターロック化は閾値を設定に持つ別フェーズ | 現状は常に safe を返す。v2: 改名は破壊的なので互換エンドポイントと廃止予定を置く |
| D4 | Phase 3c の名前付き位置の置場 | **別ファイル `layout.yaml`**（gitignore、`layout.example.yaml` を公開） | 安全限界と実験対象の位置を混同しない。v2: ZIF-8 の例は相対移動のみで `move_xyz` が無いので、移行の前提を直した |
| D5 | 本計画書の置場 | **commit**（`docs/superpowers/plans/`） | 既知の欠陥を一次ファイルの隣に置く |
| D6 | 装置追加の指針の形 | **機械検査で強制**＋**雛形生成スクリプト**。読み手は AI エージェントと人の両方 | v2: 整合テストは構造の検査であり、装置の動作契約と人のレビュー項目を置き換えない。生成は共有の読取り装置に限定して始める |

---

## 共通契約（Phase 1・2 の着手前に決める。各 1 ページの設計メモとして書く）

| ID | 契約 | 決めること | 使う項目 |
|---|---|---|---|
| C1 | **拒否と故障の例外分類** | `ValidationError`（`src/devices/safety/validators/exceptions.py:11`、`WorkspaceViolationError` はその派生）を「未送信の拒否」の基底とし、新しい拒否はすべてこの派生で投げる。`LabRobot` の `except ValidationError: raise` / `except Exception: _recover_home()`（`lab_robot.py:637-645`）と `ExperimentSession.run()`（`experiment_session.py:669-686`）の分岐はこの型で行う。汎用 `ValueError` による拒否は禁止（拒否直後に復帰動作が走る） | 1a, 1c, 0-1 |
| C2 | **停止結果の状態** | 停止要求を「要求した／装置が応答した／静止を確認した／未確認」の 4 状態で表し、run の状態（`completed` / `aborted` / `failed`）とは別に記録する。`emergency_stop()` が失敗をログに留めて正常 return した場合、session はそれを「停止済み」として `_estopped_ids` に登録してはならない（`experiment_session.py:540-560`） | 1b, 1e, 2b |
| C3 | **実効設定と無効化の意味論** | 「実効設定」= 既定 → `config.yaml` → env → CLI/GUI/CSV の順で解決し、**factory が装置へ渡した値**まで含む（`default_robot_factory`（`experiment_session.py:139-155`）は `robot_options` を MaxArm にしか渡さない）。装置の無効化は「キーを消す」では表せない（deep merge で既定が残る）ので、`enabled: false` 等の明示的な表現を決める。文字列の数値・引用符付き robot ID・部分 workspace の互換方針を決める | 2a, 2c, 3b |
| C4 | **来歴の保存許可項目** | `provenance.json` に書くのは許可リストの項目だけ。接続先（VISA、IP、Bluetooth、シリアルポート）は論理装置 ID に置き換える。依存は配布名と版のみ、URL・絶対パスは除外。「完全再現」ではなく「来歴の部分記録」と明記 | 2a |
| C5 | **イベント形式** | 版番号付き JSON Lines。ステップの開始・完了・失敗・中断、停止結果（C2）、資源の取得・解放、出力の復元、run 終了を持つ。CSV 列に無い生データ（`scope_raw`、`scope_settings_after`、発振器の変更前状態と復元結果、`experiment_logger.py:277` 付近）も載せる。末尾の書き込み途中行の復旧規則を決める。CLI JSON・GUI・CSV runner の 3 経路で同じ形式 | 2b |

---

## Phase 0: 下地（独立項目。安全層ファイルに触れるのは 0-6 のみ）

| ID | 項目 | 根拠（現状） | 変更 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 0-1 | **可動域の反例テストを置く（今は通ってしまうことを固定）** | `lab_robot.py:433` rotate は joint1 のみ、`:570` move_xyz は XYZ のみ、`:621-627` move_radial は `r+distance<0` を拒否しない。`_validate_home_legs`（`:994-1012`）はホームの Z 上昇・J1・XYZ を検査するが、現在半径で回転した途中姿勢は検査しない。mock も同構造（`mock_robot.py:132,170,193`） | `tests/test_workspace_pose_invariants.py` を新設。**原点の両側を含む箱**（例: X −250..250、Y −50..50、Z 0..100、J1 ±135）で次を `MockLabRobot` と `LabRobot`（指令列を記録する fake controller 注入）の両方に書く。(a) `move_xyz(200,0,50)` → `rotate(90)`: Y=200 で箱外になるのに通る。(b) 円弧の極値: X 上限 199.9 の箱で `(200cos2.5°, −200sin2.5°)` から `rotate_relative(5)`: 両端は箱内、0° で X=200 が違反。(c) `go_home`: `(250,0,50)` からホーム `(200,50,50)` へ帰る最初の回転で Y≈60.6 が箱外（箱 X 100..250）。(d) `move_radial(−400)`: 半径反転が X=−200 の箱内に着地して通る。(e) 拒否時に fake へ指令が 1 件も送られない。(f) 拒否時に `_recover_home` が呼ばれない。Phase 1a まで `pytest.mark.xfail(strict=True)` | (a)〜(f) が両ラッパーで xfail RED として登録され、CI が緑。各テストは送信された指令列を assert する | **M** |
| 0-2 | **完全構成の CI ジョブ** | `.github/workflows/ci.yml:22` は Ubuntu × 3.11/3.12、軽量依存（**pydobot は入っている**）。GUI AppTest は Streamlit 不在でスキップ、CSV `--mock` 煙テストが無い | ジョブ 1 本追加（3.12）。インストールは `requirements.txt` 全量 **＋ `src/monitoring` の依存**。`run_csv --mock`、GUI AppTest を含める。環境変数（例 `SAIGEN_CI_FULL=1`）が立っているときは `importorskip` をスキップでなく失敗にする conftest を置く（対象: `test_gui_app_params.py`、`test_dobot_controller_fixes.py`、`test_dobot_patch.py`、`test_force_stop.py`、`test_csv_recorder.py`） | 追加ジョブが緑で、かつスキップ件数 0 がジョブの出力で確認できる | **M** |
| 0-3 | **pydobot の版を固定** | `requirements.txt:29` は `>=1.3.2` 上限なし。`pydobot_patch.py` は 1.3.2 の private メソッドを置換する。v1 の「uv が 1.0.1 を引いた」は、下限なしの `--with pydobot` を使った実行スクリプト側の事象で、`>=1.3.2` の下では起きない（記述を訂正） | `pydobot==1.3.2`（または対応版の集合を `pydobot_patch.py` に定義し requirements はその範囲）。パッチの先頭で版を検査し、集合外なら起動時に明示エラー | 版が集合外のときのエラーメッセージをテスト。置換対象の private メソッドが存在することもテスト | S |
| 0-4 | **利用者の config.yaml を読むテストを隔離する** | `tests/test_config.py` の 3 テスト（`:18 test_robot_ports_have_int_keys`、`:37 test_robot3_has_no_pipette`、`:42 test_workspace_limits_are_published_values`）が実在の `config.yaml` を読む。限界を狭めた研究室では失敗する | 3 テストとも config の読込（キャッシュ含む）を monkeypatch し `config.example.yaml` または既定値を対象にする | 一時ディレクトリに狭い `config.yaml` を置いても 3 テストが通る | S |
| 0-5 | **古い記述の是正** | (a) `docs/csv-runner.md:126-132` 「セル外の値は腕に指令する前に失敗する」と読めるが、相対移動は preflight では未検証扱い。(b) 同文書 `:49,227` が JSON フローの撹拌を例に挙げるが IKA のアクションは無い。(c) `schema.py:13-14`「Mock は最小値を検査しない」は `mock_robot.py:289` と矛盾 | 文書を現状に合わせる。(a) は「絶対目標は実行前、相対移動は --mock / 実行時に 1 手ずつ」と書き分ける | 各記述がコードと一致 | S |
| 0-6 | **残骸の除去（安全層ファイルの編集。挙動を変えない削除。人の承認を得る）** | `lab_robot.py:288` が存在しない `src.devices.powder_dispenser` を import する（`:49-82` は引数と属性） | `use_powder_dispenser` を引数ごと削除するか、受けて警告を出して無視する。`src/devices/README.md` の記述も合わせる | `LabRobot(use_powder_dispenser=True)` の挙動がテストで固定 | S |
| 0-7 | **`.env.example` の `CAMERA_INDEX`** | `.env.example:32` が `CAMERA_INDEX=0`、`run_flow.py:321` は env を config.yaml より優先 | `.env.example` ではコメントアウトし「設定すると config.yaml より優先」と注記 | `.env.example` をコピーした環境でも `--validate-only` が config の値を使う | S |
| 0-8 | **ライセンス境界の機械検査（import 文の検査）** | `detection/` を `src/`・`examples/`・`tests/` から import する箇所は 0（AST で確認）。文字列 `detection` は画像処理のコメント等に 32 行あるので、**文字列 grep は現状でも落ちる** | `tests/test_license_boundary.py`: `ast` で `src/`・`examples/`・`tests/` 下の全 `.py` を走査し、`import detection` / `from detection` / `ultralytics` の静的 import があれば失敗 | 現状で緑。意図的に `from detection import x` を足した一時ファイルで RED になることをテスト内で確認 | S |
| 0-9 | **examples の受入表** | 「examples 全部を通す」は誤り。`examples/li5640/oscillator/` は既定設定で拒否される（exit 3）のが正しく、CI（`ci.yml:85-96`）もその拒否を検査している | `docs/` または CI の一覧に「既定で通る例」「既定で拒否される例と、通すための設定」を表で置く。Phase 1a の受入はこの表に対して行う | 表の各行が CI の実行結果と一致 | S |

Phase 0 の完了条件: 全項目のテストが CI で緑。`python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と 0-9 の表どおりの `--validate-only` / `--mock` が通る。**1a の前提は 0-1 と 0-9 のみ。** 他は並行。

---

## Phase 1: 安全層（C1・C2 を決め、人が「直す」と決めてから着手）

目的: 設定した限界が、**終点と回転の円弧**について、どのコマンドの組み合わせでも守られること。停止が確実に伝わり、結果が記録されること。一般の XYZ 直線経路や障害物との非衝突は対象外（この範囲を文書にも書く）。

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 / リスク |
|---|---|---|---|---|---|
| 1c | **エラー後の自動ホーム復帰を既定で無効に（1a と同時または先行）** | `lab_robot.py:976` `_recover_home` が 6 か所の移動失敗後に呼ばれる（`:452,504,551,589,643,678`）。**`ExperimentSession.run()` も一般例外で `go_home_all()` を呼ぶ**（`experiment_session.py:679`）。ピペット・共有測定器・記録の失敗でも対象で、LabRobot 内の復帰の後に二度目が走る場合もある | 設定キー `robots.<id>.recover_home_after_error: false`（既定）。復帰方針を LabRobot 内と session の両方に適用する。復帰しないときは **停止要求を出してから切断**（C2 の状態を記録）。`default_robot_factory` が Dobot にも設定を転送するように直す（現状は MaxArm のみ `robot_options` を渡す）。GUI・CSV runner・CLI の 3 経路で factory まで届くことを統合テストで確認。MaxArm は現行どおり自動復帰しない（共通の `true` で解除しない）。フローの明示的な `go_home` とエラー復帰は別物として扱う | 既定で `_recover_home` も `go_home_all` も呼ばれないテスト、`true` で呼ばれるテスト、3 経路の統合テスト。`tests/test_experiment_session.py:78 test_error_triggers_go_home_not_emergency_stop` と `tests/test_safety2_session.py:64 test_plain_error_still_homes_robots` を新方針に更新 | **M**。リスク: 既定の変更。実験途中で止まった腕を人が片付ける運用が前提になる。README と `config.example.yaml` に明記（D2） |
| 1a | **終点と回転円弧で全制約を検査する** | 0-1 の反例 (a)〜(f) | **安全契約**（受入条件に全部入れる）: (1) 各移動で「指令 → 移動後の姿勢 → `validate_xyz` と `validate_joint1` の両方」を 1 つのヘルパ（例 `_validate_target_pose(x, y, z, j1)`）に集約。(2) rotate / rotate_relative は現在半径を保った円弧として、**両端＋区間内の 90° 倍数の極値角**を `validate_xyz` に通す。(3) `go_home` は Z 上昇・J1 回転・XYZ 移動の各区間について、**現在半径での回転の終点を含めて**最初の指令の前に全部検査する（`_validate_home_legs` を拡張）。(4) `move_radial` は `r + distance <= 0` を `WorkspaceViolationError`（C1）で拒否。(5) Dobot の姿勢配列は J1 が `pose[4]`、R が `pose[3]`。XYZ から J1 を求める規則（`atan2(y, x)`、原点近傍の扱い、実測 J1 との対応）を文書化。(6) 拒否時は指令を 1 件も送らず、復帰動作を呼ばない。(7) **MaxArm には適用しない**。MaxArm は別座標系・逆運動学で、`validators/maxarm.py:27` の箱・半径・到達判定と `plan_home_legs`（`maxarm_robot.py:103`）を維持。(8) preflight（`run_flow.py:394`）はロボット別の予測状態をループ展開後の順に更新し、絶対目標から推定できる相対移動だけ検査する。絶対 `rotate` だけでは半径・高さは確定しないので未確定のままにする。実行時の実測検査は省かない | 0-1 の (a)〜(f) がすべて PASS（xfail を外す）。既存 `tests/test_workspace_*`、`test_safety2_mock_parity.py`、`test_maxarm_mock_parity.py` が緑。0-9 の表どおりに examples が通る／拒否される。通らなくなった例は「フローの側を直す」で対処し、理由を記録 | **L 寄り**。リスク: 今まで通っていたフローが拒否される（意図した挙動）。CHANGELOG に明記。bench 確認は人 |
| 1b | **Dobot の停止処理を送信境界まで閉じる** | 競合は fake controller で再現済み: 姿勢取得中に `emergency_stop()` を入れると現行は `STOP → rotate(10)` の順で送る。`LabRobot` は mixin を使わず、`_estop_generation` をワーカー起動直前（`lab_robot.py:355`）に取るだけ | `LabRobot(InterruptibleDriverMixin)` にし、**LabRobot 自身の `_run_blocking`（`:318`）・`_wait_worker`（`:374`）・`_read_pose`（`:386`）を削除**して mixin のものを使う。`_force_stop_arm` / `_driver_pose` / `_driver_reports_idle` / `_pose_indices`（J1 も比較する）を Dobot 向けに実装。各公開操作の入口で世代を採取し、姿勢取得・各ホーム区間・ワーカー起動・後待機まで同じ世代を渡す。**ワーカー内で送信直前に再確認**する（pydobot のパッチ `pydobot_patch.py:258` は `wait=True` の送信で停止フラグを解除するので、mixin の世代確認だけでは閉じない）。Dobot には MaxArm の `moving` に相当する状態 API が無いので、静止確認は姿勢の再読出しで行い、`stop_requested` を静止確認と混同しない。停止通信の失敗は C2 の「未確認」として返す | `threading.Event` で割込み位置を固定した fake で、次の各位置に estop を入れ、**送信列**を assert: 姿勢読出し中／読出し後、世代確認後ワーカー実行前、姿勢再読出し後送信前、停止後に遅れて戻るワーカー、二度目のキャンセル、停止通信失敗。いずれも STOP の後に移動指令が送られないこと | **L**。bench 確認は人 |
| 1d | **監視 API と安全インターロックを区別する** | `src/monitoring/dashboard/launch_sensor_dashboard.py:1121-1123` の `/is_safe` と `/api/is_safe` は常に `{"safe": true}`。`gui/runner.py:179-227` `check_is_safe` は到達不能でも通す。CLI・CSV にはゲートが無い。文言は `src/gui/README.md:45,61`、`docs/setup.md:250`（英語 "safety gate"）、GUI の「安全確認ゲート」、docstring の `safety gate` / `interlock` | 第一段（D3）: 新名称（例 `/api/sensor_status`）を追加し、**`/is_safe` と `/api/is_safe` は互換エンドポイントとして残して廃止予定を記す**（旧 GUI が 404 で実行を拒否するのを防ぐ）。`check_is_safe` の利用者も移行。GUI 表示を「センサー監視（インターロックではない）」にし、上記 3 文書と docstring の表現を直す。第二段（別フェーズ）: 閾値を `src/monitoring/config.yaml` に持つ本物のインターロック。開始拒否だけでなく実行中の検知と通信断時の挙動を定義 | 第一段: 新旧エンドポイントの両方がテストで通り、上記 3 文書と GUI 文字列に「安全ゲート／safety gate」が残らないことを列挙して確認（文字列 0 件を受入にしない）。第二段: 閾値超過で開始拒否・実行中中断・通信断時の既定の各テスト | 第一段 S〜M、第二段は別設計 |
| 1e | **停止未確認の扱いを全装置へ広げる（新規）** | `LabRobot` も `MaxArmRobot` も停止失敗をログに留めて正常 return し、session は `_estopped_ids` に登録して再停止を省く（`experiment_session.py:540-560`）。Dobot の一般移動例外は停止を試みず復帰・切断へ進む。通常 cleanup の失敗は握りつぶされる経路がある（LI5640 の出力復元だけは run を failed にする） | C2 の 4 状態を `emergency_stop()` の戻り値（または例外）で表し、session は「確認済み」だけを `_estopped_ids` に入れる。送信済みか不明な移動の例外では、復帰の前に停止要求を出す。cleanup の失敗を run 結果と GUI/CLI の表示に反映する | 停止通信失敗 → 再停止が送られるテスト。cleanup 失敗 → run が `completed` にならないテスト | M |
| 1f | **Picus2 の中断・故障経路（新規）** | USB の `flush/write/readline` は async 関数内で同期実行（`picus2_controller.py:320`）。Bluetooth の END 待ちは渡された timeout を使わない。初期化の再試行（`lab_robot.py:232`）は motor 設定に失敗した旧インスタンスを閉じずに置き換える。吸引・分注は公称時間の待機で完了扱い | 旧インスタンスを閉じる。timeout を使う。「公称時間の待機」を記録上「完了確認なし」と区別する（C2・C5）。アーム停止確認と同列には扱わない | 再試行で旧インスタンスの close が呼ばれるテスト。timeout が伝わるテスト | S〜M |

Phase 1 の完了条件: 0-1 の反例がすべて拒否される。全テスト緑。0-9 の表どおりに examples の `--mock` が通る／拒否される。人が bench で 1a（回転と go_home の拒否）と 1b（Stop）を確認した記録を `docs/*-commissioning.md` に残す。

---

## Phase 2: 再現性（C3〜C5 を決めてから）

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 2a | **実行記録に来歴を残す** | `experiment_logger.py` の `metadata.json` に git SHA・dirty・依存の版・実効 config が無い。実効 config には env から転記された VISA 接続先・IP・Bluetooth アドレスが入り、未知キーも保持される | run 開始時に `provenance.json`（git SHA、dirty の有無、依存の配布名と版、C3 の実効設定を **C4 の許可リストで直列化**したもの、env 由来の上書きキー名）。対象は CLI だけでなく GUI の編集値・CSV パラメータ・factory 内部の既定値 | 実行ディレクトリに開始直後からある。接続先・キーの値・絶対パス・URL が入らないことを、わざと秘匿値を置いた config で確認するテスト | M |
| 2b | **逐次ジャーナル** | 構造化記録は `finalize()`（`experiment_logger.py:375`）でまとめて書く。`DispenseAccuracyLogger` は既に行ごとに flush しているので「全部失われる」ではない。CSV runner はステップ中断の記録を持たない | C5 の形式で `events.jsonl` に追記＋flush。CSV / metadata / summary は同じ変換から生成し、CSV runner の `results.csv` も派生物にする。3 経路（CLI JSON・GUI・CSV runner）で同じ中断記録 | **finalize を通らずに終了する子プロセス**でそれまでのイベントが残るテスト（例外を投げるだけでは `finally` が動くので不可）。flush と電源断耐性は区別して書く。既存成果物との同等性テスト | M〜L |
| 2c | **実効設定の表示と型付き検証** | `config.py:110` の deep merge でキーを消しても既定が残る。`.env` の上書きが見えない | `run_flow.py --show-config` で C3 の実効設定と由来（default / config.yaml / `config.example.yaml` / env / CLI / GUI）を表示。pydantic モデルは **既存の全設定を受け入れる**: `shared_devices.microscope_resolution`、`mock_start_pose`（robot ID 辞書・"x,y,z,r" 文字列）、`robots.<id>.home_position`、`pose_settle_reads`、`pump_off_after_estop`、文字列の数値、引用符付き robot ID、部分 workspace。未知キー・型違いは起動時エラー | 未知キー `robots.1.prot` がエラーになるテスト。上記の既存設定が全部通るテスト。`--show-config` に由来列と「factory へ渡した値」がある | **L 寄り** |
| 2d | **CI を Windows / 3.13 に広げる** | `ci.yml:22` Ubuntu × 3.11/3.12 のみ | matrix に `windows-latest` × 3.12 と ubuntu × 3.13 を追加（軽量依存で可） | 追加ジョブが緑 | S〜M |

---

## Phase 3: 分離・保守性

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 3a | **動作表と装置表を置き、派生物を生成する** | `eeba6f5`（measure_scope）は `src/` の Python 12 本、`8b3d1a0`（Keithley）は 28 ファイルに触れた。schema・executor の集合・`ROBOT_ACTIONS_BY_TYPE`・GUI パレット・LLM プロンプトが別々 | **動作表** `src/flow/actions.py`: 動作名・schema モデル・種別（robot / shared / control / utility）・対応ロボット種別・必要資源・明示的な dispatch 対応・GUI/LLM への公開範囲。**装置表** `src/flow/devices.py`（名称は仮）: 装置名・config キー・README・examples・提供する動作。executor の集合、`plan_resources()`、GUI パレット、LLM プロンプトの一覧を表から生成。表に入れないもの: 可動域・停止・復元・出力上限・プロトコル・測定値の意味（安全ロジックは表に置かない）。動的 `getattr` による任意メソッド実行はしない | 3g の構造テスト。次に装置を足すとき触るファイルが減ることを `git show --stat` で確認 | 3g と合わせて **M〜L** |
| 3g | **装置追加の指針を機械検査で支える（置き換えではない）** | `docs/adding-an-instrument.md` のチェックリスト 12 項目に `llm_service.py`・`plan_resources()`・`experiment_session.py`・`gui/runner.py` が無い。**本文には `:216`（plan_resources と session）、`:297`（GUI runner と設定転送）、`:326`（LLM）で説明がある**ので、問題は本文とチェックリストの不一致 | `tests/test_action_registry_consistency.py`（構造の整合）: (1) schema の Union **25 種類**（loop_start / loop_end を含む）がすべて動作表にある、(2) 動作表の動作がすべて executor でディスパッチされ `plan_resources()` の資源に対応する、(3) GUI パレットと LLM プロンプトの一覧が「公開範囲」列と一致、(4) `result_keys` が `experiment_logger` の列定義にある（`weight → weight_g` 等の名前変換表を持つ）、(5) 装置表の config キーが `DEFAULTS` と `config.example.yaml` に存在する、(6) 装置表の README と examples が存在する。**これらで検査できないもの**（ドライバ本体、安全ラッパー、session、CLI/env、CI、GUI 編集部、診断・復元記録）は、装置ごとの動作契約テストと人のレビュー項目として残す。チェックリストは「構造テスト／契約テスト／人のレビュー」の 3 区分に直し、本文と一致させる | 既存 25 動作で緑。1 動作を表から消すとどの箇所が未登録かテスト名で分かる | 3a と同時 |
| 3h | **雛形生成スクリプト（共有の読取り装置に限定して始める）** | 追加の押し間違いは手順の多さから来る。`MockSharedDevices` を編集するので安全層ファイルに触れる | `scripts/new_instrument.py <name> --action <action>`（`--kind shared` のみ。robot は対象外）。生成先は新規ファイルのみ（`src/devices/<name>/`、`tests/test_<name>.py`、`examples/<name>/`）。既存ファイル（`MockSharedDevices`、schema、動作表、executor）は**上書きせず、差分を表示して人が適用**する。名前・パス・衝突を検証。生成した雛形は未実装なら明示的に失敗し、ダミー値で成功扱いしない。import やテスト収集で装置に接続しない。出力装置の上限・停止・復元を自動的に「実装済み」にしない。未接続箇所（schema、Union、dispatch、factory）を一覧で出力 | 一時ディレクトリに生成 → `pytest` が所定の箇所だけで落ちる → 雛形を埋める → 緑、を CI で再現。既存ファイルが変更されないテスト | M（robot・出力装置まで含めるなら L、別件） |
| 3i | **ガイドの書き直し（AI と人の両方向け）** | 現行は散文の手順書で、人だけの作業が本文に混ざる | 3 部構成: (A) 機械が検査する手順（3h → 3g の構造テスト → 装置の契約テスト → `--validate-only` / `--mock`）、(B) 人だけがやること（実機 self-check、commissioning ノート、出力装置の限界値の決定、bench 確認）、(C) 設計の理由（二層構造、出力装置の規則、やってはいけないこと）。**(C) を生成の前に読む導線**を (A) の先頭に置く（テストが緑でも出力装置の安全規則は省けない）。`docs/templates/commissioning.md` を置き既存 4 本（MaxArm・Keithley・LI5640・DSO1000）と揃える。**CLAUDE.md と AGENTS.md の両方**を更新。チェックリストの修正と (C) は 3h を待たず先に書く | Keithley・LI5640・DSO1000 の追加を新ガイドに当てはめ、抜けがないことを表で確認 | S〜M |
| 3b | **LLM プロンプトの装置像を設定から生成** | `llm_service.py:56-62` に「Dobot 3 台と任意の MaxArm、Robot 3 はピペットなし」が固定 | C3 の無効化表現が決まった後、実効設定の robots から robot_id の説明を、動作表から動作一覧を生成 | 無効化した robot が説明に出ないテスト（**「2 台だけ書いた config」では現行の deep merge で既定の 3・4 が残るので、テストは C3 の表現で書く**） | S（C3 と 3a の後） |
| 3c | **名前付き位置を前処理で解決する** | フローに位置の参照が無く座標を各ステップに書く。**ZIF-8 の例 3 本は相対移動のみで `move_xyz` が無く**、開始姿勢に依存する | `layout.yaml`（D4）に `positions: {<id>: {robot: <id>, x, y, z}}`、単位 mm、原点はロボットの座標系、layout の探索場所と版（内容ハッシュ）を定義。`move_xyz` に `position: <id>` を許し `x/y/z` との併用は拒否。実行前に数値フローへ変換し、元フロー・layout の版・変換後フローを実行ディレクトリに保存。**ZIF-8 の移行は前提が違うので本項の受入にしない**: 開始姿勢とホームの意味を定義した後、別件で検討 | 名前と数値の併用が拒否される。未知名・robot 不一致が拒否される。変換後のフローが既存の検証をそのまま通る。前処理は純粋関数として 1a の前にテストできる | M〜L |
| 3d | **GUI でフロー JSON を保存** | `gui/app.py:1494` に読込はあるが保存が無い | `st.download_button`。「設計途中の保存」を許すか「保存物は schema 適合」を要求するかを決める（推奨: 適合しないときは警告付きで保存を許す） | AppTest でダウンロードの内容が schema を通る | S |
| 3e | **`wait` をロボット不要にする** | `schema.py:222` `wait` はロボット操作扱いで省略時 Robot 1 を要求 | 独立した sleep 処理にし、資源不要（robot も `SharedDevices` も要求しない）として動作表の種別 utility に置く。`wait_after_*` は装置の安定化待ちなので残し、フローの `wait` へ一律に移す説明はしない | ロボット無し・共有装置無しのフローで `wait` が通るテスト | S（3a の種別と調整） |
| 3f | **CSV runner を 3c の上に載せる** | `run_csv.py:267` の手順は Python に固定 | 3c の後、JSON テンプレート＋パラメータ置換で表現できるか検討。**できなくても可** | 現行 CSV で生成される座標列と一致 | L（任意） |

---

## Phase 4: 公開物の整合

| ID | 項目 | 根拠 | 変更案 | 規模 |
|---|---|---|---|---|
| 4a | 論文の主張・データ・解析・コード版の対応表 | 公開物だけでは粒径と分注速度の関係を再解析できない。分注安定性 CSV は再集計で n=100、RSD 0.74 %（0.7375… の丸め） | `examples/zif8/README.md` に表: 主張 / 根拠ファイル / 解析コード / コミット / 公開していないもの。生データは「無い」と明記（規則 6） | S |
| 4b | CITATION に DOI と preferred-citation | 別セッションが作業ツリーで追加中 | その作業に合流（本計画では触らない） | — |

---

## 順序と依存（v2）

```
0-1 反例テスト ──┐
0-9 examples 表 ─┤
C1 例外分類 ─────┼──▶ 1c ──▶ 1a ──▶ bench（人）
C2 停止状態 ─────┤         （1c と 1a は同時でも可。1c を後にしない）
                 └──▶ 1b（1a と並行可。送信境界の契約は C2 と共有）──▶ bench（人）
                      1e（C2 の後、1b と並行）、1f（独立）
1d 第一段: 独立（互換エンドポイントを置く条件で先行可）
0-2〜0-8: 独立。安全修正の前提にしない
C3・C4・C5 ──▶ 2a ∥ 2b（イベント形式と run ID を合意すれば並行）、2c（C3 の後）、2d 独立
3a＋3g（同時）──▶ 3h ──▶ 3i(A)        3i(C) とチェックリスト修正は先行
C3 ──▶ 3b（3a の後）
3c: 前処理の設計と純粋関数テストは随時。統合は 1a の後
3e: 3a の種別と調整。3d 独立
Phase 4: 随時
```

- 本線は 0-1 → C1/C2 → 1c → 1a。他は並行。
- 3a/3g が Phase 1 と並行できるのは「安全に無関係だから」ではなく、**明示的な dispatch 対応と資源転送の契約を維持する条件**で。3h の `MockSharedDevices` 編集は Phase 1 の変更箇所と調整する。
- 各フェーズの終わりに Astra の反証レビュー（read-only の `codex exec`）を 1 回。指摘はコードで裏取りしてから採用する。

## 規模（v2）

| 項目 | v1 | v2 | 理由 |
|---|---|---|---|
| 0-1 | S | M | ホーム・円弧の極値・無送信・無復帰まで含む |
| 0-2 | S | M | 依存解決、監視系の追加、非スキップ確認 |
| 1a | M | L 寄り | ホーム各区間、円弧、予測状態、例外分類、実機/mock/preflight の一致 |
| 1b | M〜L | L | mixin 移植に加えて送信境界と停止結果の契約 |
| 1c | S | M | session・factory・3 経路・既存テスト |
| 1d | S / M | S〜M / 別設計 | 互換エンドポイント。第二段は設計から |
| 1e, 1f | — | M, S〜M | 新規 |
| 2a | S〜M | M | 許可リスト直列化と実効設定の解決 |
| 2b | M | M〜L | 成果物の同等性、復旧、中断・復元情報 |
| 2c | M | L 寄り | 既存設定の互換と由来追跡 |
| 3a＋3g | M＋M | M〜L | 構造テストだけなら M、契約保証を含めればそれ以上 |
| 3h | M | M（shared 限定） | robot・出力装置を含めれば L |
| 3c | M〜L | M〜L | ZIF-8 移行を外した |

## 進め方の規約（全フェーズ共通）

- 1 項目 = 1 ブランチまたは 1 連のコミット。失敗するテストを先に書き、RED を見てから実装（TDD）。安全の反例テストは「拒否された」でなく「送信された指令列」を assert する。
- 完了報告の前に `python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と、0-9 の表どおりの `--validate-only` / `--mock` を実行する。
- `src/devices/safety/` を編集する項目（0-6、1a、1b、1c、1e、1f、3h）は着手前に人の承認を得る。limits を緩める方向の変更は一切行わない。拒否が増える変更は CHANGELOG と README に明記する。
- 新しい拒否は必ず `ValidationError` の派生で投げる（C1）。
- 実機を動かす確認（1a の回転と go_home、1b の Stop）は人が行い、結果を `docs/*-commissioning.md` に書く。AI は `--mock` までで止まる。
- 各項目の実装手順書は着手時に `docs/superpowers/plans/YYYY-MM-DD-<id>-<slug>.md` として書く。共通契約 C1〜C5 は `docs/superpowers/specs/YYYY-MM-DD-<id>-<slug>.md` に 1 件 1 ファイル。

## 今回は含めないもの

- `gui/app.py`（1,604 行）・`shared_devices.py`（1,603 行）・`lab_robot.py`（1,269 行）の分割。1b・3a で自然に減る分を見てから判断。
- 共通実行サービスへの分割。3a・3c が終わると `run_flow.py` の役割が明確になるので、その後に再評価。
- インストールの軽量化（LLM・Streamlit・Torch を extras に分ける）。0-2 の完全構成 CI ができてから別件で。
- Dobot の種類追加（`config.py:194` の 2 種類固定）。要望が出てから。
- 1d 第二段（本物のインターロック）の設計。閾値の決定は人の作業。
- 3h のロボット種別・出力装置への拡張。
- ZIF-8 の例を layout 参照版に移行すること（開始姿勢とホームの意味の定義が先）。
