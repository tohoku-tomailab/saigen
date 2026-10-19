# Agilent 1000 シリーズ デジタルオシロスコープ（USB、読み取り専用）

DSO1002A / 1004A / 1012A / 1014A / 1022A / 1024A（2 または 4 チャネル）。背面の
USB DEVICE ポート（USBTMC）で、VISA ライブラリ（NI-VISA など）を通して PyVISA
で話す。セットアップは `docs/setup.md` §12。コマンドは
"Agilent 1000 Series Oscilloscopes Programmer's Guide"（版 011.04.00.04.06、
2010 年 4 月）で確認した（以下の p.n はその頁）。

## 方針: 設定は前面パネル、SAIGEN は自動測定値を読むだけ

垂直（V/div、オフセット、プローブ倍率、結合、帯域制限）、水平（s/div）、トリガ、
取り込み（ノーマル / アベレージ / ピーク）は前面パネルで設定する。SAIGEN が送るのは
問い合わせだけ（例外は接続・切断時のデバイスクリアと接続時の `*CLS`）で、設定を書き込む形のコマンド（`*RST`、`:AUToscale`、`:RUN` /
`:STOP` / `:SINGLE`、`:CHANnel` / `:TIMebase` / `:TRIGger` / `:ACQuire` /
`:MEASure` の設定形）は送らない。副作用は接続・切断時のデバイスクリア（入出力
バッファを消す）と接続時の `*CLS`（状態とエラーキューを消す、p.60。前面操作で溜まっていたエラーも
消える）だけ。GTL は送らない: DSO1014A（ファームウェア 00.04.04）と NI-VISA の USB
では `control_ren` が 120 秒止まった末に `VI_ERROR_SYSTEM_ERROR` になった。問い合わせの
接続の合間には前面パネルで設定を変えられた（`:KEY:LOCK?` = `ENABLE` は「前面パネル
有効」、p.135）。ただし GTL が止まった接続の後に一度、前面パネルがロックされていた。
原因は確定していない（前面から解除した。Force キーか電源の入れ直しかは記録していない）。

## 配線と設定

- 背面の USB DEVICE（四角いコネクタ）と PC を USB ケーブルでつなぐ（p.21）
- リソース文字列は `USB0::0x0957::<製品 ID>::<シリアル>::INSTR`。シリアル番号を
  含むので `config.yaml` にだけ書く:

  ```yaml
  shared_devices:
    scope_resource: "USB0::0x0957::<製品ID>::<シリアル>::INSTR"
    scope_timeout_ms: 5000
    wait_after_scope: 0.0      # 各 measure_scope の後の待機
  ```

## コマンド列

```
接続:     デバイスクリア, *CLS, *IDN?（機種が対応表にあること。チャネル数を覚える）,
          :SYSTem:ERRor?（0 まで、最大 31 回）
各ステップ: :TIMebase:FORMat?（X-Y なら失敗。応答は Y-T / X-Y / ROLL、p.214）
          設定スナップショット（問い合わせのみ）
          :MEASure:<項目>? CHANnel<n> を項目ごとに、samples 回（interval 秒おき）
          設定スナップショット（もう一度）, :SYSTem:ERRor?（項目があれば失敗）
切断:     デバイスクリア → close（クリアが失敗しても閉じる。GTL は送らない）
```

スナップショット: 時間軸 FORMat / MODE / SCALe / OFFSet、チャネルの DISPlay /
SCALe / OFFSet / PROBe / COUPling / BWLimit / FILTer / INVert / VERNier / UNITs、
ACQuire TYPE / AVERages、TRIGger MODE / COUPling / HOLDoff / SENSitivity /
HFREject と（EDGE のとき）EDGE SOURce / LEVel / SLOPe / SWEep、`:MEASure?`（前面
自動測定のロック）、`:KEY:LOCK?`（ガイドは `ENAB` / `DIS`、実機は `ENABLE` を返した。
どちらも受け付けて `ENAB` / `DIS` に揃える）。値の範囲はガイドのとおりに検査する
（時間軸 1 ns〜50 s/div、ホールドオフ 100 ns〜1.5 s、トリガ感度 0.1〜1 div）。`:MEASure:SOURce?`・`:TRIGger:STATus?`・
`:ACQuire:SRATe?` は記録だけで、前後で比べない。

ステップが失敗する条件: 時間軸 X-Y または DELAYED（ズーム。測定窓を設定から
決められない、p.217）、トリガがエッジ以外（その方式の設定を読まないので変更を
検出できない）、チャネル非表示、機種にないチャネル、前後で設定が違う（途中で変えて
戻された場合は検出できない）、要求した項目のどれかで有効な値が 0 回、I/O エラー、
読めない設定値、測定後のエラーキューに項目。

## 測定項目と単位

| 項目 | ヘッダー | 単位 |
|---|---|---|
| `vpp` `vmax` `vmin` `vamplitude` `vtop` `vbase` `vaverage` `vrms` | VPP ほか | チャネルの UNITs（V / A / W、UNKNown は空） |
| `frequency` | FREQuency | Hz |
| `period` `risetime` `falltime` `pwidth` `nwidth` | PERiod ほか | s |
| `pdutycycle` `ndutycycle` `overshoot` `preshoot` | PDUTycycle ほか | %（応答に % を含む、p.168, 172, 174, 178） |

捨てる応答（項目ごと・理由ごとに数える）: `<` / `>` で始まる境界値（立ち上がり
時間などで返る、p.165, 181）、読めない文字列、NaN / Inf、大きさ 9.9e37 以上
（ガイドではカウンタの頁 p.103 だけに書かれているが、:MEASure でもチャネル非表示の
とき `9.9E+37` が返ることを実機で確かめた）。% は % の
4 項目だけ末尾の 1 文字を外す。

`samples` は読み取り回数で、新しい取り込みの回数ではない（オシロはコマンドに
応答している間は取り込まない、p.27）。1 回の読み取りの中の項目も同時ではない。

## 実機で確かめたこと（DSO1014A、ファームウェア 00.04.04、2026-10-16）

- `:MEASure:<項目>? CHANnel2` を送っても `:MEASure:SOURce?` は `CHANnel1` のまま、
  画面の測定表示も増えなかった（`:MEASure:TOTal?` = 0）
- チャネル非表示では全項目 `9.9E+37`
- `:KEY:LOCK?` は `ENABLE`（ガイドの `ENAB` と違う）。`ENAB` は前面パネル有効の意味で、
  この状態で前面から V/div を変え、次の接続でその値が読めた
- GTL は 120 秒止まって失敗（上記）
- 取り込み STOP・波形なしの状態で、接続から設定スナップショットと 3 項目の読み取り
  までの自己確認が 0.5 秒前後で終わった（取り込み中の応答時間は未確認）

## まだ確かめていないこと

- 方形波などの信号を取り込んだ状態での読み取り。実機で見たのは、STOP のまま残っていた
  平らな取り込み（CH1 0.5 V/div、50 ms/div）で、Vpp が `4.000e-002`（0.04 V、有効値として
  読めた）、周波数とデューティ比が `9.9E+37`（エッジがないので測れない）だけ
- GTL が止まった後に前面パネルがロックされていた原因と、その解除方法

- % の位置と `<` の現れ方
- 前面自動測定のロック（`:MEASure?` = Locked）で問い合わせに応答するか
- ROLL 表示で `:TIMebase:OFFSet?` がエラーにならないか

## 接続確認（人が実行する。接続・切断時のデバイスクリアと接続時の `*CLS` のほかは問い合わせだけ）

```bash
python -m src.devices.dso1000.dso1000 USB0::0x0957::<製品ID>::<シリアル>::INSTR [channel]
```

導入の記録（実機の応答、実機とレビューで見つかった不具合、残っている確認事項）は
`docs/dso1000-commissioning.md`。
