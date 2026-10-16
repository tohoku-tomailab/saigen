# NF LI5640 ロックインアンプ（GPIB、読み取り専用）

エヌエフ回路設計ブロックのマルチファンクション・デジタル・ロックインアンプ。
SAIGEN は GPIB（IEEE 488.1、488.2 一部準拠）で PyVISA を通して読む。RS-232 には
対応しない（LI5640 は GPIB と RS-232 を同時には使えない）。セットアップは
`docs/setup.md` §11。コマンドは取扱説明書（英語版 DB509993-003）第 5 章で確認した
（以下の 5-n は説明書のページ）。

## 方針: 測定の設定は前面パネル、SAIGEN は読むだけ

感度・時定数・スロープ・参照信号・入力・フィルタ・ダイナミックリザーブ・
DATA1 / DATA2 の表示・オフセット・拡大・正規化・レシオは前面パネルで設定する。
SAIGEN がこれらを送ることはない。送る設定は読み出し項目の選択 `OTYP`（リモート
専用、5-22, 5-32）だけで、接続時の値を覚えておき、切断時に戻して読み返す。

そのほかの副作用:

- デバイスクリア（この機器宛ての SDC。入出力バッファを消す。設定は変わらない、5-77, 5-78）
- `*CLS`（イベントレジスタとエラーキューを消す。戻さない、5-36）
- GTL までの REMOTE 状態（前面パネルを操作できない、5-6）。GTL はこの機器宛てで、
  バス全体の REN・IFC・DCL は使わない

## 配線と設定

- 前面 SHIFT+SPECIAL の `intFc` を `GPib` にする（5-3）
- 前面 SHIFT+ADDRESS でアドレス（0〜30）を設定する（5-4）。同じバスのほかの機器と
  重ならないこと（両方のパネルで確かめる）。送信ターミネータはどれでもよい
  （ドライバは END で受信を終え、CR / LF を削る）
- 連続データ出力（`OSTR 1`）は止めておく。動いていると他の問い合わせが通らない
  （5-68 の -440）ので、接続時に `OSTR?` が 1 なら初期化を失敗にする（止める
  コマンドは送らない）
- `config.yaml`:

  ```yaml
  shared_devices:
    lockin_resource: "GPIB0::<アドレス>::INSTR"
    lockin_timeout_ms: 5000
    wait_after_lockin: 0.0     # 各 measure_lockin の後の待機（測定前の整定待ちではない）
  ```

## コマンド列

```
接続:     デバイスクリア, *CLS, *IDN?（LI5640 を含むこと）, EROR?（0 まで、最大 21 回）,
          OSTR?（0 であること）, OTYP?（覚える）
各ステップ: OTYP 1,2,3,5 → EROR?        DOUT? が DATA1, DATA2, FREQ, OVERLEVEL を返す
          設定スナップショット（問い合わせのみ）
          DOUT?, WRCR? を samples 回（interval 秒おき）
          設定スナップショット（もう一度）
切断:     デバイスクリア → OTYP <覚えた値> → OTYP?（読み返し）→ GTL → close
          （どれかが失敗しても次へ進む。中断要求は見ない）
```

スナップショットで読む設定: `DDEF? 1/2`, `ISRC?`, `NORM?`, `VSTD?` / `ISTD?`,
`RAT?`, `KFAC?`, `OFSO? 1/2`, `OFFS? 1/2`, `OEXP? 1/2`, `PHAS?`, `ICPL?`, `IGND?`,
`VSEN?` / `ISEN?`, `TCON?`, `SLOP?`, `SYNC?`, `NOIS?`, `ILIN?`, `IFRQ?`, `ITHR?`,
`DRSV?`, `RSRC?`, `REDG?`, `HARM?`, `OPCR?`, `OSTR?`。読めない・範囲外の応答は
ステップの失敗。前後で違えば失敗（途中で変えて戻された場合は検出できない）。
`OPCR` の 256 / 512（自動設定・自動感度の実行中）も失敗。

`FREQ?` は内部発振器の設定周波数で、測定周波数ではない（5-23）。測定周波数は
`DOUT?` の FREQ（高調波測定では基本波）で読む。`DOUT?` と `WRCR?` は同時刻ではない。

## 値の意味（`describe()`）

DATA1 / DATA2 は**パネル表示の値**（5-70）。

| 条件 | DATA1 | DATA2 |
|---|---|---|
| 通常 | X / R: V（入力 A, A−B）または A（I 入力）。AUX IN1: V | Y: V または A。θ: deg。AUX: V |
| DATA1 = NOISE | V/√Hz または A/√Hz（4-35） | DDEF に関係なく AUX IN1、V（4-34） |
| 正規化 dB / % | X / R が dB / %（4-32） | Y への効き方は説明書から読めないので種類 `Y[normalize?]`、単位なし |
| レシオ表示 | X / R が比（単位なし、正規化より優先、4-33） | Y が比。θ・AUX はそのまま |

オフセット・拡大は単位を変えないが、値は素の X / Y ではなくなる。設定は
`lockin_settings` に残る。

## 捨てるサンプル（保守的な方針）

次のどれかがあればサンプル全体（AUX のチャネルも含む）を捨て、理由ごとに数える:
OVERLEVEL の 1（入力）・2（中段）、表示中チャネルの 4 / 8（レシオ表示なら 16 / 32）、
`WRCR` の 1（UNLOCK）・2（SIGNAL 参照で同期できない）、`DOUT?` の形が違う・
非有限、`WRCR?` が 0〜3 の整数でない。オーバーレベル時の数値は説明書にないので、
値（Keithley の 9.9E37 のような閾値）では判定しない。

中央値は有効サンプルで取る。θ は全点が幅 180° 未満の円弧に入るときだけ、その円弧の
上で中央値を取る（±180° をまたいでも順序に依存しない）。入らなければ失敗。

## 整定

SAIGEN は整定を待たない。パネルを変えたら、ステップの前に人が待つ（説明書
表 4-3: 24 dB/oct で 99 % に約 10τ、99.9 % に約 13τ。NOISE は 24 dB/oct で
約 170τ × 平滑化係数、4-34）。`interval` は読み取りの間隔で、整定待ちではない。

## 使用例と接続確認

```python
from src.devices.li5640 import LI5640, READ_OUTPUT_TYPE, describe

lia = LI5640("GPIB0::<アドレス>::INSTR")
saved = None
try:
    lia.clear_status()
    print(lia.identify())
    saved = lia.output_type()
    lia.set_output_type(READ_OUTPUT_TYPE)
    snap = lia.snapshot()
    print(describe(snap), lia.read_sample())
finally:
    lia.release(saved)        # クリア・OTYP 復元・GTL・切断
```

実機で確かめたこと・未確認のこと（ファームウェア 1.60 の応答、レビューで見つかった
不具合、残っている確認事項）は `docs/li5640-commissioning.md`。

接続確認（人が実行する。読むだけで、OTYP は戻す）:

```bash
python -m src.devices.li5640.li5640 GPIB0::<アドレス>::INSTR
```
