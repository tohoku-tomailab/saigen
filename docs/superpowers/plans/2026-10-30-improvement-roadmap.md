# SAIGEN 改善計画（2026-10-30、改訂 2026-10-31 v3.1）

対象: `ba5641f`。`102e617` 以降に入ったソース変更は `src/devices/mg400/` だけなので、本書が引用する安全層・フロー層の行番号は `102e617` 版と同じ。根拠は Codex（gpt-6-astra）による評価（2026-10-21 プロジェクト全体、2026-10-30 「Python とフロー設計の分離」、2026-10-30 本計画書 v1 の反証レビュー、2026-10-31 v2 の再評価と `src/` の 4 観点の分析）と、その指摘を Claude がコードで裏取りした結果。評価の出力はセッションの一時領域にしかないので、根拠は本書に file:line で写してある。

この文書は「何を・どの順で・どこまで」を決める計画である。各項目の実装手順書（タスク分割・テスト・コミット単位）は、着手するときに `docs/superpowers/plans/` に別途書く。

## 改訂履歴

- v1（2026-10-30、`102e617`→`4e4d566`）: 初版。
- v2（2026-10-31、`ba5641f`）: v1 への反証レビューの 5 点を反映。1a の安全契約、session と送信境界への拡張（1e/1f 追加）、Phase 0 の成立しない検査の修正、共通契約 C1〜C5、3a〜3i の保証範囲。
- v2 の再評価（2026-10-31）: 「3a〜3i の保証範囲」は反映済み、他の 4 件は一部反映。安全層の実装は保留とされ、次の 5 件を求められた。(1) 停止と送信の原子性と停止後の新操作の禁止、(2) 検証した具体的な指令をそのまま送ること、(3) C1・C2・1c・1e を一つの異常終了契約にすること、(4) 0-1 を実行できるテスト仕様にすること、(5) 設定転送の許可リストと 1f の範囲・依存。
- v3（2026-10-31）: 上の 5 件を反映。(1) 共通契約 **C6「送信と停止の排他」**を新設し、1b はその実装とした。(2) 1a に「検証した具体的な指令をそのまま送る」と preflight の予測無効化を加えた。(3) 旧 1c と旧 1e を **1c「異常終了の契約」**に統合し、C1・C2 を拡張した。(4) 0-1 を開始姿勢・4 要素ホーム・境界値つきの表にし、一律の strict xfail をやめた。(5) C3 に factory 転送の許可リストを入れ、1f を Picus2 の実 API と依存に合わせた。ほかに再評価が挙げた不整合（C2 の行番号、C3 の設定フォールバック、C5 の録画情報、`pydobot_patch.py` の行番号、0-2 の非スキップ方式、0-3 の未検証の逸話、0-6 の挙動、0-9 の表、1d の文書一覧、3d の受入、3g の種別ごとの検査）を直した。
- v3.1（2026-10-31）: `src/` 分析で見つかった**天秤とピペットの不整合**を編入。共通契約 **C7「計量と分注の対応」**と **Phase 2W（W1〜W6）**を追加し、「今回は含めないもの」から該当分を外した。安全層の契約（C1〜C6、Phase 0・1）は v3 から変えていない。Astra の v3 再評価は v3.1 の追加分を対象にしていない。

## 方針

1. **安全 → 再現性 → 分離（保守性）の順。** ただし全項目をこの順に直列化はしない。反例テストと、安全層に触れない文書・CI の是正は並行して進める。
2. **安全層の変更は人の決定が要る。** `src/devices/safety/` を編集する項目は本書で明示する（0-6、1a、1b、1c、1f、3h、W3）。反例テストを先に置き、人が「直す」と決めてから実装する。AI は限界を緩めない。
3. **反例を先にテストにする。** 実機ラッパーと mock は同じ構造を共有しているので、mock との一致テストでは検出できない。制約そのものに対する反例を、両方に独立に置く。実機ラッパーは fake controller が記録した**送信列**で、mock は**状態が変わらないこと**で確かめる。
4. **検証したものを送る。** 安全検査は、装置へ送る具体的な値に対して行う。検証の後で下位の層が状態を読み直して目標を計算し直す経路を残さない（1a・C6）。
5. **拒否と故障を例外の型で分け、拒否は副作用の前に置く。** 「送信前に検証で拒否した」ことと「送信後に装置が失敗した」ことは復帰の扱いが違う。前者で復帰動作を走らせてはならない（C1）。
6. **予測は実測の代わりにならない。** preflight の姿勢推定は補助であり、実行時の実測姿勢に対する検査を省く根拠にしない。
7. **実機での確認は人の作業。** 回転の経路や停止の挙動はコードから導いた推論であって実測ではない。Phase 1 の受入には bench での確認が要る。

## 決定済み（2026-10-30、AskUserQuestion で確認）

| # | 決定 | 採用 | 理由・補足 |
|---|---|---|---|
| D1 | Phase 1a の検査範囲 | **終点＋回転経路** | 固定刻みの点検査は保証にならない（半径 200 mm、−2.5°→+2.5°、X 上限 199.9 mm で両端 X≈199.81 は通り、0° で X=200 が違反）。**両端に加えて、回転区間内で X・Y が極値をとる角度（区間に入る 90° の倍数）を必ず検査する**。原点中心・半径一定・Z 一定の円弧と軸平行の箱では、これで網羅できる（v2 再評価で検算済み） |
| D2 | エラー後の自動ホーム復帰 | **既定で無効**（`recover_home_after_error: true` で opt-in） | 液体・治具を保持した状態で動いてよいかは研究室ごとに違う。復帰しない場合は「停止要求 → 停止の確認 → 切断」。**停止が未確認なら opt-in でも復帰しない**（C2） |
| D3 | センサーの `is_safe` | **まず位置づけ直し**。インターロック化は別フェーズ | 現状は常に safe を返す。改名は破壊的なので互換エンドポイントと廃止予定を置く |
| D4 | 名前付き位置の置場 | **別ファイル `layout.yaml`** | 安全限界と実験対象の位置を混同しない。ZIF-8 の例は相対移動のみで `move_xyz` が無いので移行は対象外 |
| D5 | 本計画書の置場 | **commit**（`docs/superpowers/plans/`） | 既知の欠陥を一次ファイルの隣に置く |
| D6 | 装置追加の指針の形 | **機械検査で強制**＋**雛形生成スクリプト**。読み手は AI と人の両方 | 整合テストは構造の検査であり、装置の動作契約と人のレビュー項目を置き換えない。生成は共有の読取り装置に限定して始める |

---

## 共通契約（Phase 1・2 の着手前に決める。1 件 1 ファイルの設計メモにする）

| ID | 契約 | 決めること | 使う項目 |
|---|---|---|---|
| C1 | **拒否・故障の分類と送信記録** | (a) 送信前の拒否はすべて `ValidationError`（`validators/exceptions.py:11`）の派生で投げる。**既存の拒否も棚卸しする**: `move_radial` の原点近傍の拒否は `ValueError`（`lab_robot.py:615`）で、汎用例外の枝に落ちてホーム復帰を呼ぶ（`:643`）。(b) **拒否は副作用の前に置く**: `rotate_relative` は速度プリセットを装置へ書いてから検証している（`:476-478` の後に `:487` で検証）。拒否時にも書込み 2 回が送られるので、検証を先にする。(c) 各公開操作は「この操作で何を送ったか」（なし／設定のみ／移動を送信）を記録し、異常時の処理はこの記録で決める。複数区間の操作（`go_home`）で途中の区間が拒否・失敗した場合も、送信済みの区間とワーカーの残存で判断する。(d) 測定器の失敗（天秤・DMM の有効値なし `shared_devices.py:545,656` など）がロボットの復帰を起こすかを明示的に決める（現状は `ExperimentSession.run()` の一般例外の枝 `experiment_session.py:679` で全ロボットがホームへ動く） | 1a, 1c, 0-1 |
| C2 | **停止の状態と確認** | 停止を「要求した／装置が応答した／静止を観測した／未確認」で表し、run の状態（`completed` / `aborted` / `failed`）とは別に記録する。**「確認済み」の定義**: 装置の停止応答に加え、姿勢の再読出しで静止を観測したこと。観測の条件（読出し回数・間隔・許容差）と限界を書く。現行の静止判定（`lab_robot.py:386-400`、隣り合う 2 回の差が許容内）は、既定の 0.2 秒／0.5 mm では 2 mm/s で動き続ける腕も通る。確認は**停止世代と接続**に紐付け、再接続・新しい世代で失効させる。session は確認済みのロボットだけを `_estopped_ids` に入れる（現行は `emergency_stop()` が正常 return すれば登録する、`experiment_session.py:563`）。未確認なら再停止を期限付きで試み、それでも未確認なら自動復帰を禁止し、GUI/CLI と run 記録に「停止未確認」を出す | 1b, 1c, 2b |
| C3 | **実効設定、無効化、転送** | 「実効設定」= 既定 → `config.yaml`（**無ければ `config.example.yaml`**、`config.py:154`）→ env → CLI/GUI/CSV の順で解決し、**factory が装置へ渡した値**まで含む。装置の無効化は「キーを消す」では表せない（deep merge で既定が残る）ので、`enabled: false` 等の明示的な表現を決める。文字列の数値・引用符付き robot ID・部分 workspace の互換方針を決める。**factory への転送は許可リストにする**: `robot_options()`（`experiment_session.py:69`）は除外キー以外をすべて返すので、Dobot へ単純に `**robot_options(ports)` を渡すと `dobot_homing`（`lab_robot.py:193`）が届き、初期化時のファームウェアホーミングを起動し得る。転送するキーを列挙し、`dobot_homing` は人が明示した場合以外通さない | 1c, 2a, 2c, 3b |
| C4 | **来歴の保存許可項目** | `provenance.json` に書くのは許可リストの項目だけ。接続先（VISA、IP、Bluetooth、シリアルポート）は論理装置 ID に置き換える。依存は配布名と版のみ、URL・絶対パスは除外。「完全再現」ではなく「来歴の部分記録」と明記 | 2a |
| C5 | **イベント形式** | 版番号付き JSON Lines。ステップの開始・完了・失敗・中断、送信記録（C1）と停止の状態（C2）、資源の取得・解放、出力の復元、**録画の情報と所有**（現行 metadata の `sensor_recording`、`experiment_logger.py:454`、と「このランが開始した録画か」）、run 終了を持つ。CSV 列に無い生データ（`scope_raw`、`scope_settings_after`、発振器の変更前状態と復元結果）も載せる。末尾の書き込み途中行の復旧規則を決める。CLI JSON・GUI・CSV runner の 3 経路で同じ形式 | 2b, 1f |
| C6 | **送信と停止の排他（新設）** | (a) **停止ラッチ・世代確認・指令送信を同じ排他規則で直列化**する。確認と送信が別々なら、確認を何回足しても間に STOP が入る。現行の pydobot パッチは `wait=True` の送信で停止フラグを解除してから送る（`pydobot_patch.py:241-243`）。v2 再評価では、最後の世代確認の直後に STOP を入れると `STOP → MOVE` が送られ、例外も出ないことを fake で再現した。(b) **送信と完了待ちを分ける**: 排他は送信だけを囲み、完了待ちの間は保持しない（STOP を塞がない）。(c) **停止後の新操作の禁止**: STOP の後は、明示的な再開（人の操作、または session の作り直し）までラッパーが新しい操作を拒否する。操作の入口で「現在の世代」を採るだけでは、STOP の後に始まった操作が通る（v2 再評価で `STOP → 新しい move_xyz → キャンセル → DISCONNECT` を再現）。(d) 停止フラグの解除は「再開」の操作だけが行い、送信のたびには解除しない | 1b, 1c |
| C7 | **計量と分注の対応（v3.1 新設）** | (a) `measure_weight` の値の意味: 最後の `tare_scale` からの正味重量（天秤の表示値）。tare の無いフローでは「基準なし」。(b) **分注誤差を自動で計算してよい条件**: 「tare → dispense 1 回 → measure_weight」の順で、間に別の dispense も tare も無いときだけ。条件外の計量も行は残すが、誤差を空欄にして理由（`multiple_dispenses_since_tare`、`no_tare`、`tare_after_dispense`、`no_dispense`）を書く。(c) **1 回の tare の後に複数回分注する手順**（プリセット 2 本）の扱い。案 1: 直前の計量との差分で誤差を出す。容器が天秤に載ったままで、分注先が天秤上の容器だとフローで宣言したときだけ。案 2: 計算しない。**推奨は「既定は計算しない。差分はフローで宣言したときだけ」**。分注先はフローからは分からないので推定しない。宣言の形（例: `measure_weight` の任意フィールド `accuracy_basis: "since_tare" \| "since_previous" \| "none"`）は人が決める。(d) 密度: 現行はラン共通の 1 値（`--liquid-density` 等）。密度の違う 2 液のために、ロボット別または dispense 別の密度を許すか。(e) 単位と安定表示: 天秤の値は g に限る。g 以外と不安定表示の扱い。(f) 集約: 「中央値」の定義（偶数個のとき中央 2 値の平均か上側か）と、有効読取数・生の読取値の記録 | W1〜W4, 2b |

---

## Phase 0: 下地（独立項目。安全層ファイルに触れるのは 0-6 のみ）

| ID | 項目 | 根拠（現状） | 変更 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 0-1 | **可動域と拒否の反例テスト** | 下表の反例。いずれも現行の `LabRobot`（fake controller）と `MockLabRobot` で通る、または拒否後に動作が出ることを v2 再評価で確認 | `tests/test_workspace_pose_invariants.py` を新設。**各反例の中に「拒否される」「fake への移動指令が 0 件」「復帰が呼ばれない」を含める**。現行で既に成立する保証（例: 既に拒否される `rotate(180)` で送信 0 件）は通常のテストにし、xfail を付けない。xfail(strict=True) は「現行で通ってしまう反例」だけに付ける。実機ラッパーは fake の送信列、mock は姿勢と保持量が変わらないことで確かめる | 下表の xfail がすべて RED として登録され、通常テストが緑。CI が緑 | M |
| 0-2 | **完全構成の CI ジョブ** | `.github/workflows/ci.yml:22` は Ubuntu × 3.11/3.12、軽量依存（pydobot は入っている）。GUI AppTest は Streamlit 不在でスキップ、CSV `--mock` 煙テストが無い | ジョブ 1 本追加（3.12）。インストールは `requirements.txt` 全量 ＋ `src/monitoring` の依存 ＋ **Node.js**（`src/monitoring/tests/test_dashboard_static.py:43` の `skipif`）。`run_csv --mock`、GUI AppTest を含める。**非スキップの確認**: `importorskip` はモジュール収集時に呼ばれるので、autouse fixture では遅い。`conftest.py` の `pytest_collectreport` と `pytest_runtest_logreport` で、環境変数 `SAIGEN_CI_FULL=1` のときにスキップを失敗へ変える。対象は `test_gui_app_params.py`・`test_dobot_controller_fixes.py`・`test_dobot_patch.py`・`test_force_stop.py`・`test_csv_recorder.py`・`test_dashboard_static.py` | 追加ジョブが緑。対象ファイルの 1 つで依存を外すとジョブが失敗することを一度確かめる | M |
| 0-3 | **pydobot の版を固定** | `requirements.txt:29` は `>=1.3.2` 上限なし。`pydobot_patch.py` は 1.3.2 の private メソッドを置換する。v1 の「uv が 1.0.1 を引いた」は**未検証**（当時の実行スクリプトの依存指定に下限が無かった可能性が高い。`>=1.3.2` の下では起きない） | `pydobot==1.3.2`（または対応版の集合を `pydobot_patch.py` に定義）。パッチの先頭で版を検査し、集合外なら起動時に明示エラー | 版が集合外のときのエラーをテスト。置換対象の private メソッドが存在することもテスト | S |
| 0-4 | **利用者の config.yaml を読むテストを隔離する** | `tests/test_config.py` の 3 テスト（`:18`、`:37`、`:42`）が実在の `config.yaml` を読む | 3 テストとも config の読込（キャッシュ含む）を monkeypatch し、`config.example.yaml` または既定値を対象にする | 一時ディレクトリに狭い `config.yaml` を置いても 3 テストが通る | S |
| 0-5 | **古い記述の是正** | (a) `docs/csv-runner.md:126-132`「セル外の値は腕に指令する前に失敗する」と読めるが、相対移動は preflight では未検証。(b) 同文書 `:49,227` が JSON フローの撹拌を例に挙げるが IKA の動作は無い。(c) `schema.py:13-14` と `run_csv.py:247` の「Mock は最小値を検査しない」は `mock_robot.py:289` と矛盾 | 文書を現状に合わせる | 各記述がコードと一致 | S |
| 0-6 | **残骸の除去（安全層ファイルの編集。人の承認を得る）** | `lab_robot.py:288` が存在しない `src.devices.powder_dispenser` を import する。現行は `use_powder_dispenser=True` で初期化が失敗する | 引数は残し、`True` なら「粉体ドライバは公開版に無い」という明示的なエラーで初期化を止める（**失敗するという挙動は変えず、メッセージだけ明確にする**）。存在しない import は消す | `LabRobot(use_powder_dispenser=True)` が明示エラーになることをテストで固定 | S |
| 0-7 | **`.env.example` の `CAMERA_INDEX`** | `.env.example:32` が `CAMERA_INDEX=0`、`run_flow.py:321` は env を config.yaml より優先 | `.env.example` ではコメントアウトし「設定すると config.yaml より優先」と注記 | `.env.example` をコピーした環境でも config の値が使われる | S |
| 0-8 | **ライセンス境界の機械検査（import 文）** | 文字列 `detection` は 32 行あるが、`detection`／`ultralytics` の静的 import は 0 件（AST で確認） | `tests/test_license_boundary.py`: `ast` で `src/`・`examples/`・`tests/` の全 `.py` を走査し、静的 import があれば失敗。動的 import は対象外と明記 | 現状で緑。テスト内で一時的な `from detection import x` を解析させると RED | S |
| 0-9 | **examples の受入表と CI** | 公開既定値で 9 例中 8 例が preflight を通り、発振器の例だけが既定で拒否される（exit 3）。現在の CI は 9 例すべての mock を実行してはいない | `docs/examples-matrix.md`（新設）に「例 / 既定で通るか / 通すための設定」の表を置き、CI に 9 例すべての `--validate-only` と `--mock` を表どおりに実行する手順を足す。発振器の例は既定で exit 3、CI と同じ 0.3 Vrms／10–1000 Hz の設定で通る | CI の各行の結果が表と一致 | S |

**0-1 の反例表**（箱は特記なしで X −250..250、Y −50..50、Z 0..100、J1 ±135）

| # | 反例 | 開始姿勢・設定 | 操作 | 現行 | 修正後に期待すること |
|---|---|---|---|---|---|
| (a) | 回転の終点が箱外 | `move_xyz(200, 0, 50)` の後 | `rotate(90)` | 通る | Y=200 で拒否、送信 0 件 |
| (b) | 円弧の極値 | 箱の X 上限 199.9。開始 `(199.809644, −8.723877, 50)`、J1=−2.5° | `rotate_relative(5)` | 通る | 0° で X=200 が違反として拒否 |
| (c) | ホーム復帰の途中姿勢 | 箱 X 100..250。開始 `(250, 0, 50)`、ホーム **`[200, 50, 50, 14.036243]`**（4 要素。3 要素だと現行は回転を省く、`lab_robot.py:1054`） | `go_home()` | 通る | 回転後の予測 `(242.54, 60.63, 50)` が箱外として、最初の指令の前に拒否 |
| (d) | 半径の反転 | 開始 `(200, 0, 50)` | `move_radial(−400)` | 通る（X=−200 は箱内） | `r + distance = −200 <= 0` で拒否 |
| (d') | 半径がちょうど 0 | 開始 `(200, 0, 50)` | `move_radial(−200)` | 通る | `r + distance = 0` で拒否（境界値） |
| (g) | 検証と送信のずれ | Z 上限 100。fake の姿勢読み取りが 1 回目 Z=50、2 回目 Z=90 を返す | `move_z(40)` | Z=90 を検証して Z=130 を送る | 検証した Z=90 がそのまま送られる（1a の (9)） |
| (h) | 拒否が復帰を誘発 | 開始 `(0, 0, 50)`、ホーム `[200, 0, 50, 0]` | `move_radial(10)` | `ValueError` の後 `rotate(0)` と `move_xyz(200, 0, 50)` が送られる | 拒否のみ、送信 0 件（C1 (a)） |
| (i) | 拒否の前の副作用 | 開始 J1=0 | `rotate_relative(180, speed="low")` | 拒否されるが速度プリセットの書込み 2 回が送られる | 送信 0 件（C1 (b)） |

mock の `go_home()` は途中区間を実行せず最後に姿勢をホームへ置き換える（`mock_robot.py:230`）ので、(c) の mock 側は「最初の指令の前に拒否され、姿勢が変わらない」ことだけを確かめる。

Phase 0 の完了条件: 全項目のテストが CI で緑。`python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と 0-9 の表どおりの `--validate-only` / `--mock` が通る。**1a の前提は 0-1 と 0-9 のみ。** 他は並行。

---

## Phase 1: 安全層（C1・C2・C6 を決め、人が「直す」と決めてから着手）

目的: 設定した限界が、**終点と回転の円弧**について、どのコマンドの組み合わせでも守られること。送る値が検証した値と一致すること。停止が確実に伝わり、停止後に新しい操作が始まらず、結果が記録されること。一般の XYZ 直線経路や障害物・他機との非衝突は対象外（この範囲を文書にも書く）。

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 / リスク |
|---|---|---|---|---|---|
| 1c | **異常終了の契約（旧 1c と旧 1e を統合。1a と同時または先行）** | 自動復帰の経路が 2 つある: `LabRobot` 内の `_recover_home`（6 か所、`lab_robot.py:452,504,551,589,643,678`）と、`ExperimentSession.run()` の一般例外での `go_home_all()`（`experiment_session.py:679`）。同じ失敗で両方が走り得る。停止失敗は両ラッパーともログに留めて正常 return し、session は停止済みとして登録する（`:563`）。初期化中に中断すると、未登録のロボットは STOP なしで切断だけされる（`:438-444`）。通常 cleanup の失敗は握りつぶされる経路がある | C1・C2・C6 を一つの実装単位にする。(1) 自動復帰の担い手を session に一本化し、`LabRobot` 内の `_recover_home` は呼ばない（二重復帰を防ぐ）。(2) `robots.<id>.recover_home_after_error: false`（既定）。true でも、C1 の送信記録が「移動を送信」で、C2 の停止が確認済みのときだけ復帰する。(3) 復帰しないときは「停止要求 → 停止の確認 → 切断」。(4) 初期化中・登録済み・cleanup 中のどの段階でも、接続済みの装置には切断の前に停止要求を出す。(5) MaxArm は現行どおり自動復帰しない（`NO_RECOVERY_HOME_TYPES`、共通の true で解除しない）。(6) フローの明示的な `go_home` とエラー復帰を区別する。(7) cleanup の失敗を run 結果と GUI/CLI の表示に反映する。(8) factory への設定転送は C3 の許可リストで行い、GUI・CSV・CLI の 3 経路で factory まで届くことを統合テストで確かめる | 既定で `_recover_home` も `go_home_all` も呼ばれない。true でも停止未確認なら呼ばれない。同じ失敗で復帰が 1 回だけ。初期化中の中断で STOP が送られる。停止通信の失敗で再停止が送られ、`_estopped_ids` に入らない。cleanup 失敗で run が `completed` にならない。`tests/test_experiment_session.py:78` と `tests/test_safety2_session.py:64` を新方針に更新 | **M〜L**。リスク: 既定の変更。実験途中で止まった腕を人が片付ける運用が前提になる。README と `config.example.yaml` に明記（D2） |
| 1a | **終点と回転円弧で全制約を検査し、検証した指令を送る** | 0-1 の (a)〜(d')・(g) | **安全契約**（受入条件に全部入れる）: (1) 各移動で「指令 → 移動後の姿勢 → `validate_xyz` と `validate_joint1` の両方」を 1 つのヘルパ（例 `_validate_target_pose(x, y, z, j1)`）に集約。(2) rotate / rotate_relative は現在半径を保った円弧として、**両端＋区間内の 90° 倍数の極値角**を `validate_xyz` に通す（D1）。(3) `go_home` は Z 上昇・J1 回転・XYZ 移動の各区間について、**現在半径での回転の終点を含めて**、最初の指令の前に全部検査する（`_validate_home_legs` `lab_robot.py:994-1012` を拡張）。(4) `move_radial` は `r + distance <= 0` を `WorkspaceViolationError`（C1）で拒否。(5) Dobot の姿勢配列は J1 が `pose[4]`、R が `pose[3]`。XYZ から J1 を求める規則（`atan2(y, x)`、原点近傍の扱い、実測 J1 との対応）を文書化。(6) 拒否時は指令を 1 件も送らず、復帰動作を呼ばない（C1）。(7) **MaxArm には適用しない**。`validators/maxarm.py:27` の箱・半径・到達判定と `plan_home_legs`（`maxarm_robot.py:103`）を維持。(8) preflight（`run_flow.py:394`）はロボット別の予測状態をループ展開後の順に更新し、絶対目標から推定できる相対移動だけ検査する。絶対 `rotate` だけでは半径・高さは確定しないので未確定のままにする。**`go_home`、中断の可能性がある操作、姿勢を確定できない操作の後は予測を「不明」に戻す**。実行時の実測検査は省かない。(9) **検証した具体的な指令をそのまま送る**: ラッパーは検証済みの絶対値（XYZ と R、または J1〜J4 の全成分）をドライバへ渡し、ドライバは姿勢を読み直して目標を作らない。対象は `PyDobotController.move_Z`（姿勢を読み直して Z を加算、`pydobot_controller.py:224-233`）、`move_XYZ_abs` の R 補完（`:214-216`）、`move_angle` の J2〜J4 読み直し（`:245-250`）、`go_home` の各区間。(10) mock の `go_home` も区間ごとに検査して進める（現行は最後にまとめて置き換える） | 0-1 の (a)〜(d')・(g) がすべて PASS（xfail を外す）。既存の `tests/test_workspace_*`、`test_safety2_mock_parity.py`、`test_maxarm_mock_parity.py` が緑。0-9 の表どおりに examples が通る／拒否される。通らなくなった例は「フローの側を直す」で対処し、理由を記録 | **L**。リスク: 今まで通っていたフローが拒否される（意図した挙動）。ドライバの API を変える。CHANGELOG に明記。bench 確認は人 |
| 1b | **Dobot の停止を C6 で実装する** | 競合は fake で再現済み: 姿勢取得中に `emergency_stop()` を入れると現行は `STOP → rotate(10)`。世代確認の直後でも `STOP → MOVE`（C6 (a)）。`LabRobot` は mixin を使わず、`_estop_generation` をワーカー起動直前（`lab_robot.py:355`）に取るだけ。区間間の sleep の後に開始時の世代と比べない。停止タイムアウト後に残ったワーカーを次の操作から隔離する仕組みも無い | `LabRobot(InterruptibleDriverMixin)` にし、**LabRobot 自身の `_run_blocking`（`:318`）・`_wait_worker`（`:374`）・`_read_pose`（`:386`）を削除**して mixin のものを使う。`_force_stop_arm` / `_driver_pose` / `_driver_reports_idle` / `_pose_indices`（J1 も比較）を Dobot 向けに実装。各公開操作の入口で世代を採取し、姿勢取得・各区間・ワーカー起動・後待機まで同じ世代を渡す。**C6 を実装する**: pydobot パッチの送信を「停止ラッチを見て、世代を確認し、送る」の排他区間にし、`wait=True` での停止フラグ解除をやめる。完了待ちは排他の外で行う。STOP の後は再開まで新しい操作を拒否する。残存ワーカーがあれば次の操作を拒否する（mixin の `_refuse_if_worker_alive`）。Dobot には MaxArm の `moving` に相当する状態 API が無いので、静止の観測は姿勢の再読出しで行い、C2 の限界を記録する | `threading.Event` で割込み位置を固定した fake で、次の各位置に STOP を入れ、**送信列**を assert: 姿勢読出し中／後、世代確認後ワーカー実行前、**世代確認と送信の間**、完了待ちの間、区間間の sleep 中、停止後に遅れて戻るワーカー、STOP 後に始まった新しい操作、二度目のキャンセル、停止通信失敗。いずれも STOP の後に移動指令が送られない | **L**。bench 確認は人 |
| 1d | **監視 API と安全インターロックを区別する** | `src/monitoring/dashboard/launch_sensor_dashboard.py:1121-1123` の `/is_safe` と `/api/is_safe` は常に `{"safe": true}`。`gui/runner.py:179-227` `check_is_safe` は到達不能でも通す。CLI・CSV にはゲートが無い | 第一段（D3）: 新名称（例 `/api/sensor_status`）を追加し、**`/is_safe` と `/api/is_safe` は互換エンドポイントとして残して廃止予定を記す**。`check_is_safe` の利用者も移行。表現を直す対象は次の 4 か所: `src/gui/README.md:45,61`、`docs/setup.md:250`（英語 "safety gate"）、GUI の文字列「安全確認ゲート」（`gui/app.py:529,660`）、`gui/runner.py` のコメントと docstring（`:168,675` ほか、`safety gate` / `interlock`）。第二段（別フェーズ）: 閾値を `src/monitoring/config.yaml` に持つ本物のインターロック。開始拒否だけでなく実行中の検知と通信断時の挙動を定義 | 第一段: 新旧エンドポイントの両方がテストで通る。上記 4 か所の表現が直っていることを列挙して確認（文字列 0 件を受入にしない）。第二段: 閾値超過で開始拒否・実行中中断・通信断時の既定の各テスト | 第一段 S〜M、第二段は別設計 |
| 1f | **Picus2 の中断・故障経路** | USB の `flush/write/readline` は async 関数内で同期実行され（`picus2_controller.py:320`、`readline` は `:369`）、イベントループを止め得る。シリアル生成（`:192`）に `write_timeout` が無い。Bluetooth の END 待ちは渡された timeout を使わない。初期化の再試行（`lab_robot.py:232`）は motor 設定に失敗した旧インスタンスを切断せずに置き換える。吸引・分注は公称時間の待機で完了扱い | **独立にできる部分**: 再試行で旧インスタンスを `await disconnect()`（`picus2_controller.py:265`。`close()` は無い）で切断する。timeout を Bluetooth 経路へ渡す。シリアルに `write_timeout` を付ける。同期 I/O をワーカースレッドで実行し、時間上限で中断できるようにする。**C2・C5 に依存する部分**: 「公称時間の待機」を記録上「完了確認なし」として、アームの停止確認とは別に記録する | 再試行で旧インスタンスの `disconnect()` が呼ばれる（USB と Bluetooth の両方）。timeout が伝わる。同期 I/O が止まったときにキャンセルが時間上限内に効く | M |

Phase 1 の完了条件: 0-1 の反例がすべて拒否される。全テスト緑。0-9 の表どおりに examples の `--mock` が通る／拒否される。人が bench で 1a（回転と go_home の拒否、`move_z` の送信値）と 1b（Stop と停止後の新操作の拒否）を確認した記録を `docs/*-commissioning.md` に残す。

---

## Phase 2: 再現性（C3〜C5 を決めてから）

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 2a | **実行記録に来歴を残す** | `experiment_logger.py` の `metadata.json` に git SHA・dirty・依存の版・実効 config が無い。実効 config には env から転記された接続先が入り、未知キーも保持される | run 開始時に `provenance.json`（git SHA、dirty の有無、依存の配布名と版、C3 の実効設定を **C4 の許可リストで直列化**したもの、env 由来の上書きキー名）。対象は CLI だけでなく GUI の編集値・CSV パラメータ・factory 内部の既定値 | 開始直後からある。接続先・キーの値・絶対パス・URL が入らないことを、わざと秘匿値を置いた config で確認 | M |
| 2b | **逐次ジャーナル** | 構造化記録は `finalize()`（`experiment_logger.py:375`）でまとめて書く。`DispenseAccuracyLogger` は既に行ごとに flush している。CSV runner はステップ中断の記録を持たない（`run_csv.py:502` は `Exception` だけ） | C5 の形式で `events.jsonl` に追記＋flush。CSV / metadata / summary は同じ変換から生成し、CSV runner の `results.csv` も派生物にする。3 経路で同じ中断記録 | **finalize を通らずに終了する子プロセス**でそれまでのイベントが残る。flush と電源断耐性は区別して書く。既存成果物との同等性テスト | M〜L |
| 2c | **実効設定の表示と型付き検証** | `config.py:110` の deep merge でキーを消しても既定が残る。`.env` の上書きが見えない | `run_flow.py --show-config` で C3 の実効設定と由来（default / config.yaml / config.example.yaml / env / CLI / GUI）を表示。pydantic モデルは **既存の全設定を受け入れる**: `shared_devices.microscope_resolution`、`mock_start_pose`（robot ID 辞書・"x,y,z,r" 文字列）、`robots.<id>.home_position`、`pose_settle_reads`、`pump_off_after_estop`、文字列の数値、引用符付き robot ID、部分 workspace。未知キー・型違いは起動時エラー | 未知キー `robots.1.prot` がエラー。上記の既存設定が全部通る。`--show-config` に由来列と「factory へ渡した値」がある | **L 寄り** |
| 2d | **CI を Windows / 3.13 に広げる** | `ci.yml:22` Ubuntu × 3.11/3.12 のみ | matrix に `windows-latest` × 3.12 と ubuntu × 3.13 を追加（軽量依存で可） | 追加ジョブが緑 | S〜M |

---

## Phase 2W: 計量と分注の結果の意味（v3.1 追加。C7 を決めてから）

動機: 手順はデータとして自由に組めるのに、結果を解釈するコードが「分注ごとに tare する」手順を前提している。実験結果の意味に直接効く不整合なので、安全層（Phase 1）とは独立に進める。W3 以外は `src/devices/safety/` に触れない。

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| W1 | **分注精度ロガーを C7 に合わせる** | `accuracy_logger.py:49-74` は直近の dispense 1 件だけを保持し、tare を見ない。次の計量と対応付けて「期待 = 量 × 密度」で誤差を出す。プリセット `dispense_experiment.json`（3 サイクル）と `weighing_dispense_cycle.json` は「tare → R1 分注 → 計量 → R2 分注 → 計量」なので、2 回目の計量は累積重量なのに R2 の量とだけ比べられ、R1 の分（密度 1 なら 1〜5 g）が誤差として記録される。ほかに、dispense と計量の間に tare があると正味 0 g と比べる。計量しなかった dispense は黙って上書きされる。行に robot_id が無い。対応付けのコードは 3 経路に重複している（`run_flow.py:862-871`、`gui/runner.py:707-716`、`run_csv.py:513-524`）。ZIF-8 の例 3 本は dispense ごとに tare するので影響しない | 対応付けを 1 つの純粋関数にまとめる（tare・dispense・measure の事象を順に受け取る）。3 経路はこれを呼ぶ。C7 (b)(c) に従い、条件外の行は誤差を空欄にして理由を書く。列は末尾に追加する（`robot_id`、`basis`、`reason`）。既存列の名前と順は変えない。mock 実行の CSV には mock であることを書く（mock の重量は常に 4.98 g、`mock_robot.py:436`） | 純粋関数のテスト: ZIF-8 の順序では現行と同じ行。`dispense_experiment` の順序では 2 回目の行に誤差が出ない（宣言したときは差分で正しい値）。tare が間にある、計量なしの dispense が続く、tare が無い、の各ケース。3 経路の統合テストで同じ CSV。既存の `tests/test_csv_runner.py:337` と `tests/test_gui_runner.py` が緑 | M |
| W2 | **解析スクリプトと過去の記録** | `src/gui/analyze_dispense_log.py` は「各 dispense の前に tare する」前提（docstring `:19-20`）で `measured_weight_g` を目標量ごとに集計する。累積重量の行も統計に入る | 誤差が空欄で理由付きの行を集計から除き、除いた件数を表示する。README と CHANGELOG に「1 回の tare の後に複数回分注するフローで記録した過去の `dispense_accuracy.csv` は、2 回目以降の行の誤差が誤り」と書き、該当するランの見分け方（metadata のフロー名、行の並び）を示す。論文の分注安定性 CSV（4a、n=100）をどのフローで記録したかは人が確認する | 理由付きの行が統計に入らないテスト。理由列の無い旧形式の CSV も読める | S |
| W3 | **重量の集約と生データ（安全層ディレクトリの編集。人の承認を得る）** | `shared_devices.py:547-548` は並べ替えて `len//2` 番目を返す。偶数個（3 回中 1 回が読めず 2 個になる場合を含む）では上側の値になり、schema の説明「中央値」（`schema.py:684`）とも、DMM・lock-in の `statistics.median`（`shared_devices.py:658,829`）とも違う。生の読取値と有効数は結果に残らない。別系統の天秤 API（`scale_controller.py:218`、平均）もある | C7 (f) に合わせる。推奨は `statistics.median` にそろえ、結果に生の読取値と有効数を入れる（C5 のイベントと measurements の列）。測定の集約であり、可動域・停止には触れない。別系統の API は標準経路で使われていないことを確かめ、README に「標準経路ではない」と書く | `[1, 3]` で 2、`[1, 2, 3]` で 2。有効数 0 は現行どおり例外。生の読取値が記録に出る | S |
| W4 | **天秤の応答の単位と安定表示** | `BCE8221.py:33-52` の `extract_number` は最初の数値だけを取り、単位と安定・種別マーカーを捨てる。上位は g として扱う（`executor.py:178-180`）。天秤の単位設定が g 以外だと黙って誤った値になる | ドライバが値・単位・安定表示・生の応答を返し、ラッパーは g 以外を拒否する（C1 (d) の測定器の失敗として扱う）。安定表示が取れる設定なら、不安定な値を記録で区別する。**実際の応答形式は人が bench で採取**して `docs/scale-commissioning.md`（新設）に書き、採った文字列をテストの入力にする（規則 6: 形式を推測で決めない） | 採取した応答文字列から値と単位が正しく取れる。`kg`・`mg`・単位なしの応答を拒否する | S〜M（bench は人） |
| W5 | **待機時間の設定を届かせる** | `default_shared_factory`（`experiment_session.py:257-278`）は `wait_after_dmm`・`wait_after_lockin`・`wait_after_scope` を渡すが、`wait_after_scale` は渡さない（ラッパー既定 2.0 秒、`shared_devices.py:167`）。Dobot の `default_robot_factory`（`:139-155`）は `wait_after_pipette` などの待機を一つも渡さない（`lab_robot.py:126-131`） | C3 の許可リストに `wait_after_scale` と Dobot の `wait_after_*` を加え、`config.example.yaml` に現在の既定値を書く。**既定値は変えない**ので、設定していない利用者のタイミングは変わらない。実効値は `--show-config`（2c）で見えるようにする | config に値を書くと 3 経路とも factory に届く。書かなければ現行と同じ値 | S（C3 の実装に含める） |
| W6 | **ピペットの有無・既定値・生成例をそろえる** | (1) `config.example.yaml:16` は「`picus2_address` の無いロボットにはピペットが無い」とするが、preflight はこれを検査しない。`run_flow.py:710-718` は動作から Picus2 の要否を決めるだけで、実機では空のアドレスで接続を試みて初期化時に失敗する。(2) LLM プロンプトの例 2（`llm_service.py:133-144`）は吸引せずに 5 mL を分注する。実機も mock も保持量の検査で拒否する。(3) 吸引速度の省略値が、CSV は 1（`run_csv.py:78`、旧版との互換）、JSON と GUI は 5（`schema.py:266`、`app.py:108`） | (1) preflight で、`picus2_address` の無いロボットへの aspirate・dispense・blow_out を装置を開く前に拒否する（`--validate-only` の段階で分かる）。(2) 例 2 に吸引を入れる。プロンプト内の全例を schema と `--mock` に通すテストを置く（3b と共有）。(3) 差は意図どおりとして残し、`docs/csv-runner.md` と 0-9 の表に書く。実効の速度は来歴（2a）に残す | (1) `--validate-only` が拒否し、終了コードとメッセージが決まっている。(2) プロンプト例のテストが緑。(3) 文書の記述がコードと一致 | S |

Picus2 の吸引・分注が「公称時間の待機」で完了扱いになる件（`picus2_controller.py:58` の速度表）は、記録の扱いとして 1f に含めてある。

---

## Phase 3: 分離・保守性

| ID | 項目 | 根拠 | 変更案 | 受入条件 | 規模 |
|---|---|---|---|---|---|
| 3a | **動作表と装置表を置き、派生物を生成する** | `eeba6f5`（measure_scope）は `src/` の Python 12 本、`8b3d1a0`（Keithley）は 28 ファイルに触れた。schema・executor の集合・`ROBOT_ACTIONS_BY_TYPE`・GUI パレット・LLM プロンプトが別々 | **動作表** `src/flow/actions.py`: 動作名・schema モデル・種別（robot / shared / control / utility）・対応ロボット種別・必要資源・明示的な dispatch 対応・GUI/LLM への公開範囲。**装置表** `src/flow/devices.py`（名称は仮）: 装置名・config キー・README・examples・提供する動作。executor の集合、`plan_resources()`、GUI パレット、LLM プロンプトの一覧を表から生成。表に入れないもの: 可動域・停止・復元・出力上限・プロトコル・測定値の意味。動的 `getattr` による任意メソッド実行はしない | 3g の構造テスト。次に装置を足すとき触るファイルが減ることを `git show --stat` で確認 | 3g と合わせて **M〜L** |
| 3g | **装置追加の指針を機械検査で支える（置き換えではない）** | `docs/adding-an-instrument.md` のチェックリスト 12 項目に `llm_service.py`・`plan_resources()`・`experiment_session.py`・`gui/runner.py` が無い。本文には `:216`・`:297`・`:326` で説明があるので、問題は本文とチェックリストの不一致 | `tests/test_action_registry_consistency.py`（構造の整合）: (1) schema の Union **25 種類**がすべて動作表にある。(2) **種別ごとに検査する**: control（`loop_start` / `loop_end`）は `expand_loops()` で消費され dispatch されない、utility は資源を要求しない、robot / shared は executor で dispatch され `plan_resources()` の資源に対応する。(3) GUI パレットと LLM プロンプトの一覧が「公開範囲」列と一致。(4) `result_keys` が `experiment_logger` の列定義にある（`weight → weight_g` 等の名前変換表を持つ）。(5) 装置表の config キーが `DEFAULTS` と `config.example.yaml` に存在する。(6) 装置表の README と examples が存在する。これらで検査できないもの（ドライバ本体、安全ラッパー、session、CLI/env、CI、GUI 編集部、診断・復元記録）は、装置ごとの動作契約テストと人のレビュー項目として残す。チェックリストは「構造テスト／契約テスト／人のレビュー」の 3 区分に直し、本文と一致させる | 既存 25 動作で緑。1 動作を表から消すとどの箇所が未登録かテスト名で分かる | 3a と同時 |
| 3h | **雛形生成スクリプト（共有の読取り装置に限定）** | 追加の押し間違いは手順の多さから来る。`MockSharedDevices` を編集するので安全層ファイルに触れる | `scripts/new_instrument.py <name> --action <action>`（shared のみ）。生成先は新規ファイルのみ。既存ファイル（`MockSharedDevices`、schema、動作表、executor）は**上書きせず、差分を表示して人が適用**。名前・パス・衝突を検証。未実装は明示的に失敗し、ダミー値で成功扱いしない。import やテスト収集で装置に接続しない。出力装置の上限・停止・復元を「実装済み」にしない。未接続箇所を一覧で出力 | 一時ディレクトリに生成 → `pytest` が所定の箇所だけで落ちる → 埋める → 緑、を CI で再現。既存ファイルが変更されない | M（robot・出力装置まで含めるなら L、別件） |
| 3i | **ガイドの書き直し（AI と人の両方向け）** | 現行は散文の手順書で、人だけの作業が本文に混ざる | 3 部構成: (A) 機械が検査する手順、(B) 人だけがやること、(C) 設計の理由。**(C) を生成の前に読む導線**を (A) の先頭に置く。`docs/templates/commissioning.md` を置き既存 5 本（MaxArm・Keithley・LI5640・DSO1000・MG400）と揃える。**CLAUDE.md と AGENTS.md の両方**を更新。チェックリストの修正と (C) は 3h を待たず先に書く | Keithley・LI5640・DSO1000 の追加を新ガイドに当てはめ、抜けがないことを表で確認 | S〜M |
| 3b | **LLM プロンプトの装置像を設定から生成** | `llm_service.py:56-62` に装置像が固定 | C3 の無効化表現が決まった後、実効設定の robots から robot_id の説明を、動作表から動作一覧を生成 | 無効化した robot が説明に出ない（テストは C3 の表現で書く） | S（C3 と 3a の後） |
| 3c | **名前付き位置を前処理で解決する** | フローに位置の参照が無い。ZIF-8 の例 3 本は相対移動のみで `move_xyz` が無い | `layout.yaml`（D4）に `positions: {<id>: {robot: <id>, x, y, z}}`、単位 mm、原点はロボットの座標系、layout の探索場所と版（内容ハッシュ）を定義。`move_xyz` に `position: <id>` を許し `x/y/z` との併用は拒否。実行前に数値フローへ変換し、元フロー・layout の版・変換後フローを実行ディレクトリに保存。ZIF-8 の移行は対象外 | 併用・未知名・robot 不一致が拒否される。変換後のフローが既存の検証をそのまま通る。前処理は純粋関数として 1a の前にテストできる | M〜L |
| 3d | **GUI でフロー JSON を保存** | `gui/app.py:1494` に読込はあるが保存が無い | `st.download_button`。schema 不適合のときは警告付きで保存を許す | AppTest で 2 ケース: 適合するフローは保存内容が schema を通る。不適合なフローは警告が出て、保存内容が編集中の辞書と一致する | S |
| 3e | **`wait` をロボット不要にする** | `schema.py:222` `wait` はロボット操作扱いで省略時 Robot 1 を要求 | 独立した sleep 処理にし、資源不要として動作表の種別 utility に置く。`wait_after_*` は装置の安定化待ちなので残す | ロボット無し・共有装置無しのフローで `wait` が通る | S（3a の種別と調整） |
| 3f | **CSV runner を 3c の上に載せる** | `run_csv.py:267` の手順は Python に固定 | 3c の後、JSON テンプレート＋パラメータ置換で表現できるか検討。**できなくても可** | 現行 CSV で生成される座標列と一致 | L（任意） |

---

## Phase 4: 公開物の整合

| ID | 項目 | 根拠 | 変更案 | 規模 |
|---|---|---|---|---|
| 4a | 論文の主張・データ・解析・コード版の対応表 | 公開物だけでは粒径と分注速度の関係を再解析できない。分注安定性 CSV は再集計で n=100、RSD 0.74 % | `examples/zif8/README.md` に表: 主張 / 根拠ファイル / 解析コード / コミット / 公開していないもの。生データは「無い」と明記（規則 6） | S |
| 4b | CITATION に DOI と preferred-citation | 別セッションが作業ツリーで追加中 | その作業に合流（本計画では触らない） | — |

---

## 順序と依存（v3）

```
0-1 反例テスト ──┐
0-9 examples 表 ─┤
C1 拒否・故障 ───┤
C2 停止の状態 ───┼──▶ 1c 異常終了 ──▶ 1a ──▶ bench（人）
C6 送信と停止 ───┤     （1c と 1a は同時でも可。1c を後にしない）
C3 転送の許可 ───┘
                 C6 ──▶ 1b（1a と並行可）──▶ bench（人）
1f: 接続破棄・timeout・同期 I/O は独立。記録の変更は C2・C5 の後
1d 第一段: 独立（互換エンドポイントを置く条件で先行可）
0-2〜0-8: 独立。安全修正の前提にしない
C3・C4・C5 ──▶ 2a ∥ 2b（イベント形式と run ID を合意すれば並行）、2c（C3 の後）、2d 独立
3a＋3g（同時）──▶ 3h ──▶ 3i(A)        3i(C) とチェックリスト修正は先行
C3 ──▶ 3b（3a の後）
3c: 前処理の設計と純粋関数テストは随時。統合は 1a の後
3e: 3a の種別と調整。3d 独立
C7 計量と分注 ──▶ W1 ──▶ W2        W3（C7 (f) と人の承認の後）
W4: bench で応答を採取（人）──▶ 実装      W5: C3 の実装に含める
W6: 独立（(2) は 3b とテストを共有）
Phase 4: 随時
```

- 本線は 0-1 → C1/C2/C6/C3 → 1c → 1a。1b は C6 の後で 1a と並行。
- 1c（旧 1c＋旧 1e）は同じ復帰・停止経路を変えるので一つの実装単位にした。
- 3a/3g が Phase 1 と並行できるのは「安全に無関係だから」ではなく、**明示的な dispatch 対応と資源転送の契約を維持する条件**で。3h の `MockSharedDevices` 編集は Phase 1 の変更箇所と調整する。
- 各フェーズの終わりに Astra の反証レビュー（read-only の `codex exec`）を 1 回。指摘はコードで裏取りしてから採用する。

## 規模（v3）

| 項目 | v2 | v3 | 理由 |
|---|---|---|---|
| 0-1 | M | M | 反例 9 件、送信列と状態の両方の検査 |
| 0-2 | M | M | 非スキップを収集フックで判定、Node を追加 |
| 1a | L 寄り | L | (9) 検証した指令をそのまま送るためにドライバの API を変える |
| 1b | L | L | C6 の実装（送信の排他、停止後の新操作の禁止） |
| 1c | M ＋ 1e M | M〜L | 旧 1c と旧 1e を統合。初期化中の停止、二重復帰の防止 |
| 1f | S〜M | M | 同期 I/O の中断まで含む |
| W1 | — | M | 対応付けの一本化、3 経路の統合テスト |
| W2〜W6 | — | 各 S（W4 は S〜M） | W4 は bench での応答採取が前提 |
| その他 | — | v2 と同じ | — |

## 進め方の規約（全フェーズ共通）

- 1 項目 = 1 ブランチまたは 1 連のコミット。失敗するテストを先に書き、RED を見てから実装（TDD）。安全の反例テストは「拒否された」でなく「送信された指令列」を assert する。
- 完了報告の前に `python -m pytest tests/ src/monitoring/tests/ src/imaging/tests/ -q` と、0-9 の表どおりの `--validate-only` / `--mock` を実行する。
- `src/devices/safety/` を編集する項目（0-6、1a、1b、1c、1f、3h、W3）は着手前に人の承認を得る。limits を緩める方向の変更は一切行わない。拒否が増える変更は CHANGELOG と README に明記する。
- 新しい拒否は必ず `ValidationError` の派生で投げ、副作用の前に置く（C1）。
- 実機を動かす確認（1a の回転・go_home・`move_z` の送信値、1b の Stop）は人が行い、結果を `docs/*-commissioning.md` に書く。AI は `--mock` までで止まる。
- 各項目の実装手順書は着手時に `docs/superpowers/plans/YYYY-MM-DD-<id>-<slug>.md` として書く。共通契約 C1〜C6 は `docs/superpowers/specs/YYYY-MM-DD-<id>-<slug>.md` に 1 件 1 ファイル。

## 今回は含めないもの

- `gui/app.py`（1,604 行）・`shared_devices.py`（1,603 行）・`lab_robot.py`（1,269 行）の分割。1b・3a で自然に減る分を見てから判断。
- 共通実行サービスへの分割。3a・3c が終わると `run_flow.py` の役割が明確になるので、その後に再評価。
- インストールの軽量化（LLM・Streamlit・Torch を extras に分ける）。0-2 の後に別件で。
- Dobot の種類追加（`config.py:194` の 2 種類固定）。要望が出てから。
- 1d 第二段（本物のインターロック）の設計。閾値の決定は人の作業。
- 3h のロボット種別・出力装置への拡張。
- ZIF-8 の例を layout 参照版に移行すること（開始姿勢とホームの意味の定義が先）。
- 2026-10-31 の `src/` 分析で見つかった、安全層以外の不具合のうち、天秤とピペットに関わらないもの（GUI の `rotate.speed`、LLM プロンプトの `rotate` 欠落、CSV runner の撮影パスと中断記録、GUI の録画要求の認証）。**本書にはまだ編入していない**。天秤とピペットの分（累積重量の誤記録、重量の中央値、単位、待機の転送、ピペットの有無、LLM の誤った正解例）は v3.1 で Phase 2W に編入した。
