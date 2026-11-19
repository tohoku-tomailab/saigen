# 別の PC で Dobot を使って遠沈管の重さ合わせをする手順

試料の入った 50 mL 遠沈管 A に液体を 20 mL 入れ、同じく試料の入った遠沈管 B に
同じ液体を足して、A と B の重さを 0.1 g 以内にそろえる（`run_balance.py`）。
アームは Dobot Magician 1 台、ピペットはその Dobot に付けた Picus 2
（既存のホルダー `cad/pipette-holder/`）、天秤は Sartorius。

**アームが動くのは 7・11 だけ。** それ以外の手順は読み取りか、ファイルの編集だけ。
実機を動かすコマンドは、Ctrl+C を押せる端末から人が実行する（Ctrl+C = その場で停止）。

## 1. ソフトを入れる

Python 3.11 と Git を入れてから、PowerShell で:

```powershell
cd $HOME\Documents
git clone https://github.com/tohoku-tomailab/saigen.git
cd saigen
git switch feature/maxarm-tube-balancing
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

以降のコマンドはすべて `saigen` フォルダで実行する。毎回 `.\.venv\Scripts\python.exe` と
打つ代わりに、最初に `$py = ".\.venv\Scripts\python.exe"` としておくと `& $py ...` で短く書ける。

## 2. 機器をつなぐ

Dobot、Picus 2、天秤をそれぞれ USB でつなぎ、Dobot の電源を入れる。

## 3. COM ポートを調べる（読み取りだけ）

```powershell
.\.venv\Scripts\python.exe -m src.devices.dobot.read_pose --list-ports
```

| 機器 | hwid に出る VID:PID |
|---|---|
| Dobot | `1A86:55D4`（CH9102）、`1A86:7523`（CH340）、`10C4:EA60`（CP2102N） |
| Picus 2 | `24BC:2202`（シリアル番号はピペット本体のラベルと同じ） |
| 天秤 | `24BC:0010` |

Dobot らしいポートで、本体の名前と座標を読む（アームは動かない）:

```powershell
.\.venv\Scripts\python.exe -m src.devices.dobot.read_pose --port COM7
```

`device 'Dobot'` と座標が出れば、そのポートが Dobot。

> **USB bridge が `10C4:EA60 CP2102N` と出たら、ここで止める。** この変換チップの Dobot は、
> COM ポートの RTS 信号が変わると本体がリセットされる。SAIGEN の Dobot ドライバは接続時に
> RTS を変えるため、実行のたびにリセットされて座標がずれる（lab_automation_CUI の
> `docs/knowledge/20260924-dobot-rts-reset.md`）。ドライバの対策を入れてから使うこと。
> CP2102N の個体は、Silicon Labs の CP210x ドライバを入れないと COM ポート自体が出ない。

## 4. config.yaml を作る

`config.example.yaml` を `config.yaml` にコピーし、`robots:` の 1 番と `shared_devices:` を書き換える
（ほかのロボットの行は残してよい。使われない）:

```yaml
robots:
  1:
    dobot_port: COM7       # 3 で調べた Dobot
    picus2_address: COM4   # 3 で調べた Picus 2
shared_devices:
  scale_port: COM6         # 3 で調べた天秤
```

`workspace:`（Dobot の可動域）は最初は見本のままでよい。位置を決めたら狭める。

## 5. 置き方

- Dobot は、ほかの Dobot と同じ高さ 210 mm の台（`cad/robot-arm-holder/`）に載せる。
  ピペットの先が、天秤の上の遠沈管（机から約 18 cm）の口より上を通れるようにするため。
- 天秤の上にラックを置き、遠沈管 A と B が立つ位置、液だめの位置を、アームが届く範囲に置く。
- 台と天秤は触れないように離す（アームの振動で重さの読みが揺れる）。

## 6. 試しに模擬実行（機器は開かない）

```powershell
.\.venv\Scripts\python.exe -m src.flow.balance_runner.run_balance --validate-only
.\.venv\Scripts\python.exe -m src.flow.balance_runner.run_balance --mock
```

`BALANCED` で終われば、ソフトの準備はできている（見本の座標・MaxArm 用の設定で動く）。

## 7. Dobot をホーミングする（アームが大きく動く）

電源を入れ直したら、最初に一度だけ。アームが左右いっぱいに振れるので周りを空ける:

```powershell
.\.venv\Scripts\python.exe -m src.devices.dobot.home_dobot --robot 1
```

## 8. 位置を教える（アームは手で動かす。コマンドは読み取りだけ）

Dobot の前腕のボタン（アンロックボタン）を押している間は、アームを手で動かせる。
ピペットにチップを付け、チップの先を目的の場所に合わせてボタンを離し、座標を保存する:

```powershell
.\.venv\Scripts\python.exe -m src.devices.dobot.read_pose --robot 1 --save reservoir_aspirate
```

保存しておく位置（名前は自由）:

| 名前の例 | チップの先の位置 | シートの項目 |
|---|---|---|
| `reservoir_aspirate` | 液だめの中央、液面より十分下 | `reservoir_x`, `reservoir_y`, `reservoir_aspirate_z` |
| `reservoir_bottom` | 液だめの底のすぐ上 | `reservoir_min_z`（z だけ） |
| `tube_a` | 遠沈管 A の口の中央、口から 5 mm ほど入れる | `tube_a_x`, `tube_a_y`, `dispense_z` |
| `tube_b` | 遠沈管 B の口の中央、A と同じ深さ | `tube_b_x`, `tube_b_y` |
| `travel` | すべての遠沈管・液だめ・ラックより 10 mm 以上上 | `travel_z`（z だけ） |

保存した座標は `dobot_positions.json` に入る。動かしながら座標を見たいときは
`--save` の代わりに `--watch`（Ctrl+C で終了）。

> 座標は Dobot 本体に記憶された「エンドエフェクタのオフセット」で決まる。`read_pose` が表示する
> `end effector offset` が、教えたときと本番で同じであること。DobotStudio などほかのソフトで
> ツール設定を変えた、または電源を入れ直したら、位置を教え直すか、オフセットが同じか確かめる。

## 9. シートを書く

`src\flow\balance_runner\balance.example.csv` を同じ場所に `balance.csv` としてコピーし、
`robot_id` を `1` に、8 の座標を各項目に書き写す。液量（`volume_a_mL` 20）、
許容差（`tolerance_g` 0.1）などはそのままでよい。各行の意味は `note` 列と README を参照。

## 10. 確かめる（機器は開かない）

```powershell
.\.venv\Scripts\python.exe -m src.flow.balance_runner.run_balance --validate-only
.\.venv\Scripts\python.exe -m src.flow.balance_runner.run_balance --mock
```

`--validate-only` は、すべての座標が `config.yaml` の可動域に入っているかを調べる。

## 11. 本番（アームが動く。最初は水と空の遠沈管で）

1. アームを手で、すべての物より上の安全な位置に置いておく。**接続したときの位置がホームになり、
   実行の最初と最後にそこへ戻る。**
2. 実行する:

   ```powershell
   .\.venv\Scripts\python.exe -m src.flow.balance_runner.run_balance
   ```

3. 画面の指示に従って、空のラック（風袋引き）→ 遠沈管 B → 遠沈管 A の順に置き、そのたびに Enter。
4. `BALANCED: A-B = ...` で終われば成功。結果は `balance_runner\results.csv` と
   `logs\<日付>\tube_balancing_<時刻>\`（`balance_log.csv` に 1 回ごとの記録）。

`NOT BALANCED` のときは理由が表示される（0.5 mL 未満の差が残った、B が重くなった、など）。
差は手で調整する。

## 困ったとき

- その PC でも Claude Code を使うなら、`saigen` フォルダで起動して
  「`src/flow/balance_runner/SETUP_DOBOT_ja.md` の手順を手伝って」と頼む。
- 機器が応答しない: 3 のコマンドで、ポートが見えているか、別のソフト（DobotStudio など）が
  ポートを使っていないかを確かめる。
