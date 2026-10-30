# SAIGEN 改善計画（2026-10-30）

対象: `f1665b4`。根拠は Codex（gpt-6-astra）による 2 回の評価（2026-10-21 プロジェクト全体、2026-10-30 「Python とフロー設計の分離」）と、その指摘を Claude がコードで裏取りした結果。評価の出力はセッションの一時領域にしかないので、根拠は本書に file:line で写してある。

この文書は「何を・どの順で・どこまで」を決める計画である。各フェーズの実装手順書（タスク分割・テスト・コミット単位）は、フェーズに着手するときに `docs/superpowers/plans/` に別途書く。

## 方針

1. **安全 → 再現性 → 分離（保守性）の順。** 登録表や名前付き位置を入れても、可動域検査の抜けは塞がらない（Astra の優先順位も同じ）。
2. **安全層の変更は人の決定が要る。** `src/devices/safety/` の変更は CLAUDE.md 規則 2 の対象。本計画の Phase 1 は、反例テストを先に置き、人が「直す」と決めてから実装する。AI は限界を緩めない。
3. **反例を先にテストにする。** 実機ラッパーと mock が同じ構造を共有しているので、mock との一致テストでは検出できない。制約そのものに対する反例を、両方に独立に置く。
4. **実機での確認は人の作業。** 回転の経路や停止の挙動は、コードから導いた推論であって実測ではない。Phase 1 の受入には bench での確認が要る。

## 決定済み（2026-10-30、AskUserQuestion で確認）

| # | 決定 | 採用 | 理由 |
|---|---|---|---|
| D1 | Phase 1a の検査範囲 | **終点＋回転経路**（角度を刻んで XYZ を検査） | 回転は終点が箱の中でも途中で箱の外を通りうる。刻み幅は設定にせず固定（例: 5°）し、保守的に |
| D2 | Phase 1c: エラー後の自動ホーム復帰 | **既定で無効**（停止・切断を既定、`recover_home_after_error: true` で opt-in） | 液体・治具を保持した状態で動いてよいかは研究室ごとに違う。現行の挙動は README で明記して変える |
| D3 | Phase 1d: センサーの `is_safe` | **まず位置づけ直し**（名称・文書・GUI 表示）。インターロック化は閾値を設定ファイルに持つ別フェーズ | 閾値は研究室固有。現状は常に safe を返すので「安全ゲートがある」と読めるのが一番の問題 |
| D4 | Phase 3c の名前付き位置の置場 | **別ファイル `layout.yaml`**（gitignore、`layout.example.yaml` を公開） | 安全限界（workspace）と実験対象の位置を混同しない。校正の単位が違う |
| D5 | 本計画書の置場 | **commit**（公開リポジトリの `docs/superpowers/plans/`） | 既知の欠陥を一次ファイルの隣に置く |
| D6 | 装置追加の指針の形 | **機械検査で強制**（3a の表を正とする整合テスト）＋**雛形生成スクリプト**。読み手は AI エージェントと人の両方 | 手順書だけでは追加コミットと食い違う（下記 3g の根拠）。人だけがやること（実機確認・閾値決定）は別節に分ける |

---

## Phase 0: 下地（低リスク、安全層に触れない）

目的: Phase 1 以降の変更を安全に進めるための CI・依存・文書の是正。各項目は独立で、順不同。

| ID | 項目 | 根拠（現状） | 変更 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 0-1 | **可動域の反例テストを置く（今は通ってしまうことを固定）** | `lab_robot.py:433` rotate は joint1 のみ、`:570` move_xyz は XYZ のみ、`:621-627` move_radial は `r+distance<0` を拒否しない。mock も同構造 `mock_robot.py:132,170,193` | `tests/test_workspace_pose_invariants.py` を新設。仮の limits（X 100..250, Y −50..50, Z 0..100, J1 −135..135）で `move_xyz(200,0,50)` → `rotate(90)` が拒否されること、`move_radial` で `r+distance<0` が拒否されることを、`MockLabRobot` と `LabRobot`（fake controller 注入）の両方に書く。Phase 1a まで `pytest.mark.xfail(strict=True)` | xfail が 4 件以上 RED で登録され、CI が緑 | S |
| 0-2 | **完全構成の CI ジョブ** | `.github/workflows/ci.yml:22` は Ubuntu × 3.11/3.12 のみ、軽量依存。GUI の AppTest は CI でスキップ、CSV `--mock` 煙テストが無い | `requirements.txt` 全量で入れるジョブを 1 本追加（3.12）。`run_csv --mock`、GUI AppTest を含める。Windows は後回し（Phase 2 で検討） | 追加ジョブが緑。`pydobot` を含む完全構成で `tests/test_dobot_controller_fixes.py` が通る | S |
| 0-3 | **pydobot の版を固定** | `requirements.txt:29` は `>=1.3.2` 上限なし。`pydobot_patch.py` は 1.3.2 の private メソッドを置換。uv が 1.0.1 を引いた実例あり（2026-10-21） | `pydobot>=1.3.2,<1.4` に。`pydobot_patch.py` の先頭でバージョン検査し、想定外なら起動時に明示エラー | 版が違うときのエラーメッセージをテスト | S |
| 0-4 | **利用者の config.yaml を読むテストを直す** | `tests/test_config.py:42` `test_workspace_limits_are_published_values` が実在の `config.yaml` を読む。限界を狭めた研究室では失敗する | テスト内で `config.yaml` の読込を monkeypatch し `config.example.yaml`（または既定値）を対象にする | 一時ディレクトリに狭い `config.yaml` を置いてもテストが通る | S |
| 0-5 | **古い記述の是正（3 件）** | (a) `docs/csv-runner.md:122-124` 相対移動が「指令前に失敗する」と読めるが preflight は未検証扱い。(b) 同文書が JSON フローの撹拌を例に挙げるが IKA のアクションは無い。(c) `schema.py:13-14`「Mock は最小値を検査しない」は `mock_robot.py:289` と矛盾 | 文書を現状に合わせる。(c) はコメント修正 | 各記述がコードと一致。`docs/csv-runner.md` に「相対移動は --mock / 実行時に 1 手ずつ検査」と明記 | S |
| 0-6 | **残骸の除去** | `lab_robot.py:49-82` `use_powder_dispenser` が存在しない `src.devices.powder_dispenser` を参照 | 引数ごと削除（後方互換のため引数は受けて警告を出す案も可）。`src/devices/README.md` の記述も合わせる | `LabRobot(use_powder_dispenser=True)` の挙動がテストで固定 | S |
| 0-7 | **`.env.example` の `CAMERA_INDEX`** | `.env.example:32` が `CAMERA_INDEX=0`、`run_flow.py:321` は env を config.yaml より優先 | `.env.example` ではコメントアウトし「設定すると config.yaml より優先」と注記 | `--validate-only` が `.env.example` をコピーした環境でも config の値を使う | S |
| 0-8 | **ライセンス境界の機械検査** | `detection/` を `src/`・`examples/`・`tests/` から import する箇所は 0（2026-10-21 確認）。CI には検査がない | CI に `grep -rn "detection" src examples tests --include=*.py` 相当の検査を追加（0 件でなければ失敗） | 意図的に import を足した PR が落ちる | S |

Phase 0 の完了条件: 全項目のテストが CI で緑。`python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と `--validate-only` / `--mock`（zif8 の例）が通る。

---

## Phase 1: 安全層（人の決定の後に着手）

目的: 設定した限界が、どのコマンドの組み合わせでも守られること。停止が確実であること。

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 / リスク |
|---|---|---|---|---|---|
| 1a | **移動後の姿勢で全制約を検査し、回転は経路も検査する** | 0-1 の反例。`go_home` は両方を検査している（`lab_robot.py:1011-1012`）ので書き方は既にある | `LabRobot` と `MockLabRobot` の各移動で「コマンド → 移動後の姿勢を計算 → `validate_xyz` と `validate_joint1` の両方」を 1 つのヘルパ（例: `_validate_target_pose(x, y, z, j1)`）に集約。rotate / rotate_relative は現在半径を保って角度を刻み、各点を `validate_xyz` に通す（D1）。`move_radial` は `r + distance <= 0` を `ValueError` で拒否。preflight（`run_flow.py:394`）は絶対移動の後に続く相対移動を、前の絶対目標から推定して検査できる範囲で検査する（できない場合は従来どおり unverified） | 0-1 の xfail がすべて PASS（xfail を外す）。既存の `tests/test_workspace_*`、`test_safety2_mock_parity.py` が緑。examples の全フローが `--validate-only` と `--mock` で通る（通らないものは「限界が狭すぎる」ではなく「フローの側を直す」で対処し、理由を記録） | M。**リスク: 今まで通っていたフローが拒否される。** これは意図した挙動なので、CHANGELOG に明記し、実機での確認は人が行う |
| 1b | **Dobot の停止処理を `InterruptibleDriverMixin` に揃える** | `maxarm_robot.py:164` は mixin を使う。`LabRobot` は使わず、`_estop_generation` をワーカー起動直前（`lab_robot.py:354`）に取るだけ。姿勢取得とワーカー起動の間に Stop が入る競合経路が推測される | `LabRobot(InterruptibleDriverMixin)` にし、`_force_stop_arm` / `_driver_pose` / `_driver_reports_idle` を Dobot 向けに実装。`emergency_stop()` の通信失敗はログだけでなく「停止未確認」の状態として GUI / CLI に返す | `tests/test_interruptible_mixin.py` 相当のテストを Dobot の fake controller で追加。「姿勢取得とワーカー起動の間に estop」を fake で再現し、追加コマンドが送られないことを確認。停止失敗時に session が `stopped` にならないこと | M〜L。**bench での確認が必須**（人の作業） |
| 1c | **エラー後の自動ホーム復帰を既定で無効に** | `lab_robot.py:976` `_recover_home` が 6 か所の移動失敗後に呼ばれる（`:452,504,551,589,643,678`）。緊急停止後は既に抑止されている | 設定キー `robots.<id>.recover_home_after_error: false`（既定）を追加。true のときだけ現行挙動。README と `config.example.yaml` に「失敗後はその場で停止・切断」と明記（D2） | 既定でホーム復帰が呼ばれないテスト、true で呼ばれるテスト。既存テストのうち復帰を前提にするものを更新 | S。リスク: 既定の変更。実験途中で止まった腕を人が片付ける運用が前提になる |
| 1d | **監視 API と安全インターロックを区別する** | `launch_sensor_dashboard.py:1121-1123` `/api/is_safe` は常に safe。`gui/runner.py:179-227` は到達不能でも通す。CLI・CSV にはゲートが無い | 第一段（D3 の推奨）: 名称を `/api/sensor_status` 相当に変え、GUI の表示を「センサー監視（インターロックではない）」にする。文書から「安全ゲート」の表現を外す。第二段（別フェーズ）: 閾値を `src/monitoring/config.yaml`（`config.example.yaml` から複製）に持ち、超過時に `is_safe=false` を返す本物のインターロックにし、CLI / CSV にも `--require-sensor-ok` を付ける | 第一段: `check_is_safe` のテストが新名称で通り、文書に「安全ゲート」が残らない。第二段: 閾値超過で run が始まらないテスト | 第一段 S、第二段 M |

Phase 1 の完了条件: 0-1 の反例がすべて拒否される。全テスト緑。examples の `--mock` が全部通る。人が bench で 1a（回転の拒否）と 1b（Stop）を確認した記録を `docs/*-commissioning.md` に残す。

---

## Phase 2: 再現性（実行記録と設定の見える化）

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 2a | **実行記録に来歴を残す** | `experiment_logger.py` の `metadata.json` に git SHA・dirty・依存の版・実効 config が無い | `finalize()` でなく run 開始時に `provenance.json`（git SHA、`git status --porcelain` の有無、`pip freeze` 相当、実効 config、`.env` 由来の上書きキー名のみ）を書く | 実行ディレクトリに開始直後から `provenance.json` がある。秘密情報（キーの値・IP）が入らないテスト | S〜M |
| 2b | **逐次ジャーナル** | 構造化記録は `finalize()`（`experiment_logger.py:375`）でまとめて書くため、強制終了で失われる | 各ステップ完了時に `events.jsonl` へ追記（append + flush）。`finalize()` は従来の CSV / metadata を同じ内容から生成 | ステップ途中で例外を投げても、それまでのステップが `events.jsonl` にある | M |
| 2c | **実効設定の表示と型付き検証** | `config.py:110` の deep merge で、キーを消しても既定が残る。`.env` の上書きが見えない | `run_flow.py --show-config` で実効設定と各値の由来（default / config.yaml / env / CLI）を表示。`config.py` に pydantic モデルを置き、未知キー・型違いを起動時エラーに | 未知キー `robots.1.prot` が「エラー」になるテスト。`--show-config` の出力に由来列がある | M。リスク: 今まで黙って無視されていた設定の誤りがエラーになる（意図した挙動） |
| 2d | **CI を Windows / 3.13 に広げる** | `ci.yml:22` Ubuntu × 3.11/3.12 のみ。実機は Windows が多い | matrix に `windows-latest` × 3.12 と ubuntu × 3.13 を追加（軽量依存で可） | 追加ジョブが緑 | S〜M（Windows 固有の失敗を直す工数は未知） |

---

## Phase 3: 分離・保守性（分離評価への対応）

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 3a | **動作のメタデータを一か所にし、整合テストを足す** | `measure_scope` の追加コミット `eeba6f5` は `src/` の Python 12 本に触れた。schema・executor・`SHARED_DEVICE_ACTIONS`・`ROBOT_ACTIONS_BY_TYPE`・GUI パレット・LLM プロンプトが別々 | 安全ロジックは動かさない。`src/flow/actions.py` に「動作名 → モデル・対象（robot / shared）・必要資源・GUI 分類・LLM 向け 1 行説明」の表を置き、executor の集合、`plan_resources()`、GUI パレット、LLM プロンプトの動作一覧をこの表から生成。動的 `getattr` による任意メソッド実行はしない | 「schema の Union にある動作がすべて表にある」「表にある動作がすべて executor でディスパッチされる」「GUI パレットと LLM プロンプトの動作一覧が表と一致」の 3 テスト。次に装置を足すとき、触るファイルが減ることを `git show --stat` で確かめる | M |
| 3b | **LLM プロンプトの装置像を設定から生成** | `llm_service.py:56-62` に「Dobot 3 台と任意の MaxArm、Robot 3 はピペットなし」が固定 | `config.yaml` の robots（種別・ピペット有無）から robot_id の説明を生成。3a の表から動作一覧を生成 | ロボットを 2 台にした config でプロンプトに Robot 3 が出ないテスト | S（3a の後） |
| 3c | **名前付き位置を前処理で解決する** | フローに位置の参照が無く、座標を各ステップに書く。例 3 本は差 1 か所で全体を複製 | 実行時に解決せず、**実行前に数値フローへ変換する前処理**を置く: `layout.yaml`（D4）に `positions: {vial_on_balance: {robot: 2, x: .., y: .., z: ..}}`、フローの `move_xyz` に `position: vial_on_balance` を許す（`x/y/z` との併用は拒否）。変換後のフローを既存の schema → preflight → safety に通し、元フロー・layout の版・変換後フローを実行ディレクトリに保存 | 名前と数値の併用が拒否されるテスト。変換後のフローが既存の検証をそのまま通る。`examples/zif8/` を layout 参照版に書き換えても `--mock` の結果（送られる座標列）が現行と一致 | M〜L |
| 3d | **GUI でフロー JSON を保存できるようにする** | `gui/app.py:1494` に読込はあるが、設計途中の保存が無い | `st.download_button` でフロー JSON をダウンロード | AppTest でダウンロードの内容が schema を通る | S |
| 3e | **`wait` と測定後の待ち時間の置き場を揃える** | `schema.py:222` `wait` はロボット操作扱いで省略時 Robot 1 を要求。測定後の待ちは `config.yaml` の `wait_after_*` | `wait` を `robot_id` 不要の共有動作にも使えるようにする（robot 不在でも動く）。`wait_after_*` は残すが文書で「フローの `wait` を推奨」と書く | ロボット無しのフローで `wait` が通るテスト | S |
| 3f | **CSV runner を 3c の上に載せる** | `run_csv.py:267` の手順は Python に固定。表はパラメータと一部の選択肢のみ | 3c の後、CSV runner の手順を `examples/zif8/` の JSON（layout 参照版）＋パラメータ置換で表現できるか検討。できれば `build_steps()` を JSON テンプレートに置き換える。**できなくても可**（現行で十分） | 現行 CSV で生成される座標列と一致 | L（任意） |
| 3g | **装置追加の指針を機械検査で強制する** | `docs/adding-an-instrument.md` のチェックリスト（12 項目）には `llm_service.py`・`run_flow.py` の `plan_resources()`・`experiment_session.py`・`gui/runner.py` が無いが、Keithley 2002 の追加コミット `8b3d1a0` はこれらに触れた（28 ファイル）。DSO1000 の `eeba6f5` も 21 ファイル。手順書は実際の変更範囲とずれている | 3a の表を「登録の正」とし、`tests/test_action_registry_consistency.py` に次を置く: (1) schema の Union にある動作がすべて表にある、(2) 表の動作がすべて executor でディスパッチされ `plan_resources()` の資源に対応する、(3) GUI パレットと LLM プロンプトの動作一覧が表と一致、(4) 表の `result_keys` が `experiment_logger` の列定義にある、(5) 表の `config_keys` が `config.example.yaml` に存在する、(6) `src/devices/<name>/README.md` と `examples/<name>/` が存在する。ガイドのチェックリストは「このテストが落ちる箇所を直す」に置き換える | 既存 23 動作でテストが緑。1 つの動作を表から消すと、どの箇所が未登録かがテスト名で分かる | M（3a と同時に実装） |
| 3h | **雛形生成スクリプト** | 追加の押し間違いは手順の多さから来る | `scripts/new_instrument.py <name> --kind shared\|robot --action <action>` が、ドライバ (`src/devices/<name>/{__init__,<name>}.py` と README)、`MockSharedDevices` のメソッド雛形、`Action*` モデル雛形、表への 1 行、`tests/test_<name>.py` の雛形（パーサテスト＋mock 実行テスト）、`examples/<name>/` の最小フローを生成。生成直後は 3g のテストが「実装待ち」で落ちる状態にし、埋めると緑になる | 生成 → `pytest` が所定の箇所だけで落ちる → 雛形を埋める → 緑、を CI 上のテストで再現（一時ディレクトリに生成） | M（3a・3g の後） |
| 3i | **ガイドの書き直し（AI と人の両方向け）** | 現行は散文の手順書で、人だけの作業（実機確認・閾値の決定）が本文に混ざる | `docs/adding-an-instrument.md` を 3 部構成に: (A) 機械が検査する手順（3h で生成 → 3g のテストを緑にする → `--validate-only` / `--mock`）、(B) 人だけがやること（実機での self-check、commissioning ノートの記入、出力を書く装置の限界値の決定、bench 確認）、(C) 設計の理由（二層構造、出力装置の規則、やってはいけないこと）。commissioning ノートのテンプレート `docs/templates/commissioning.md`（順序・ファームウェアが受けた応答・未解決点・中断規則の節）を置き、既存 4 本の構成と揃える。CLAUDE.md の「Add a new instrument」からの参照を更新 | Keithley・LI5640・DSO1000 の追加を新ガイドの手順に当てはめ、抜けがないことを表で確認 | S〜M（3g・3h の後） |

---

## Phase 4: 公開物の整合

| ID | 項目 | 根拠 | 変更案 | 規模 |
|---|---|---|---|---|
| 4a | 論文の主張・データ・解析・コード版の対応表 | 公開物だけでは粒径と分注速度の関係を再解析できない。分注安定性 CSV は再集計で README と一致（n=100、RSD 0.74 %） | `examples/zif8/README.md` に表を追加: 主張 / 根拠ファイル / 解析コード / コミット / 「公開していないもの」。公開していない生データは「無い」と明記（規則 6） | S |
| 4b | CITATION に DOI と preferred-citation | 別セッションが作業ツリーで追加中 | その作業に合流（本計画では触らない） | — |

---

## 順序と依存

```
Phase 0 (全項目独立、順不同)  ──▶  Phase 1a ──▶ 1b
                                   │            └▶ bench 確認（人）
                                   ├─▶ 1c（独立）
                                   └─▶ 1d 第一段（独立）
Phase 2 (0-2 の後なら随時)        2a → 2b、2c 独立、2d 独立
Phase 3                           3a → 3g → 3h → 3i（Phase 0 の後なら Phase 1 と並行可）
                                  3a → 3b、3c → 3f（3c は 1a の後）、3d・3e 独立
Phase 4                           随時
```

- Phase 0 と Phase 1 の 0-1 → 1a が本線。他は並行可。
- Phase 3 のうち **3a → 3g → 3h → 3i（動作の登録表と装置追加の指針）は安全層に触れないので、Phase 0 の後、Phase 1 と並行して進められる。** 3c（名前付き位置）だけは 1a の後に始める。理由: 3c の前処理は「変換後のフローが既存の検証を通る」ことに依存し、その検証が 1a で変わる。
- 各フェーズの終わりに Astra の反証レビュー（read-only の `codex exec`）を 1 回。指摘はコードで裏取りしてから採用する。

## 進め方の規約（全フェーズ共通）

- 1 項目 = 1 ブランチまたは 1 連のコミット。各項目で、失敗するテストを先に書き、RED を見てから実装（TDD）。
- 完了報告の前に `python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と、examples の `--validate-only` / `--mock` を実行する。
- `src/devices/safety/` と `workspace` の変更は、limits を緩める方向には一切行わない。拒否が増える変更は CHANGELOG と README に明記する。
- 実機を動かす確認（1a の回転、1b の Stop）は人が行い、結果を `docs/*-commissioning.md` に書く。AI は `--mock` までで止まる。
- 各項目の実装手順書は着手時に `docs/superpowers/plans/YYYY-MM-DD-<id>-<slug>.md` として書く。

## 今回は含めないもの

- `gui/app.py`・`shared_devices.py`（各約 1,600 行）・`lab_robot.py`（1,269 行）の分割。1b・3a で自然に減る分を見てから判断。
- 共通実行サービスへの分割（Astra 推奨 9）。3a・3c が終わると `run_flow.py` の役割が明確になるので、その後に再評価。
- インストールの軽量化（LLM・Streamlit・Torch を extras に分ける）。0-2 の完全構成 CI ができてから、extras 化は別件で。
- Dobot の種類追加（`config.py:194` の 2 種類固定）。要望が出てから。
