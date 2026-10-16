# Keithley 2002 デジタルマルチメータ（GPIB）

8.5 桁 DMM。インターフェースは GPIB（IEEE-488）のみで、PC 側の GPIB アダプタと
VISA ライブラリ（NI-488.2 / NI-VISA、Keysight IO Libraries など）を通して
PyVISA で SCPI コマンドを送る。セットアップは `docs/setup.md` §10。

## 配線と設定

- GPIB ケーブルで PC の GPIB アダプタと接続する。アドレスは前面 *MENU > GPIB*
  （工場出荷時 16）
- `config.yaml`:

  ```yaml
  shared_devices:
    dmm_resource: "GPIB0::16::INSTR"   # VISA リソース文字列
    dmm_timeout_ms: 10000              # 1 回の I/O の上限
  ```

## 測定機能

| `function` | SCPI | 単位 | 最大レンジ |
|---|---|---|---|
| `dc_voltage` | `VOLTage:DC` | V | 1100 |
| `ac_voltage` | `VOLTage:AC` | V | 775 |
| `dc_current` | `CURRent:DC` | A | 2.1 |
| `ac_current` | `CURRent:AC` | A | 2.1 |
| `resistance_2w` | `RESistance` | Ohm | 1.05e9 |
| `resistance_4w` | `FRESistance` | Ohm | 2.1e6 |

NPLC は全機能 0.01〜50（AC も含む）。最大レンジと NPLC の範囲は取扱説明書
（2002-900-01 Rev. E, Table 3-10）による。

## コマンド列（`configure()` → `read()`）

```
:ABOR
:CONF:<SCPI>                    単発測定の既知状態（トリガ 3 層 IMM/1 回/遅延 0、
                                演算・バッファ・スキャン無効、その機能の設定は *RST 値）
:SENS:<SCPI>:RANG:AUTO ON       または :SENS:<SCPI>:RANG <値>
:SENS:<SCPI>:NPLC <n>
:UNIT:VOLT:AC V                 （ac_voltage のみ。dB 表示のままだと単位が変わる）
:FORM:DATA ASC
:FORM:ELEM READ                 応答は数値だけ
:FORM:EXP HPR                   レンジの工学単位に丸めず、出力時の桁落ちを抑える
:SYST:ERR?                      0 になるまで読む。エラーがあれば Keithley2002Error
:READ?                          = :ABOR + :INIT + :FETCh?（1 回変換）。samples 回
:SENS:<SCPI>:RANG? / :NPLC?     最後のサンプルの後に実際の設定を読み返して記録
                                （I/O エラーは握りつぶさない: 応答がずれるため）
```

`:CONF` が変えない機器全体の設定のうち、ライン同期（`:SYST:LSYN:STAT?`）は
初期化時に読んで run.log に残す。

`*RST` は送らない。`:CONF` は取得方式・オートゼロ・その機能の設定を既定に
戻すが、その外の設定（GPIB アドレス、表示、ライン同期など）は変えない。オーバーフローは
`+9.9E37` で返り、`read()` は None にする。終了時は GTL（GPIB のバスコマンド、
`control_ren`）で前面操作に戻す。設定は戻さない。

## 使用例

```python
from src.devices.keithley2002 import Keithley2002

dmm = Keithley2002("GPIB0::16::INSTR")
try:
    dmm.clear_status()
    print(dmm.identify())
    dmm.configure("dc_voltage", nplc=10)
    print(dmm.read())
    print(dmm.read_settings("dc_voltage"))   # 実際のレンジ・NPLC
finally:
    try:
        dmm.local()
    finally:
        dmm.close()                          # GTL が失敗しても閉じる
```

接続確認（人が実行する。機器と通信するがロボットは動かさない）:

```bash
python -m src.devices.keithley2002.keithley2002 GPIB0::16::INSTR
```

このとき runpy の `RuntimeWarning`（パッケージの `__init__` が先にモジュールを
import するため）が出るが、無害。

フローからは `measure_dmm` アクション（`SharedDevices.measure_dmm`）を使う。
I/O はワーカースレッドで行い、ロックで直列化する。中断（Ctrl+C / GUI の Stop）
ではステップはすぐに終わり（ロボットの緊急停止を待たせない）、ワーカーは
進行中の 1 回の I/O が終わったところで残りの回数を打ち切る。ハンドルを
閉じるのは常にロックを持つ側で、cleanup() が待ちきれない場合はワーカーが
通信を終えた時点で GTL・切断する（通信中に閉じることはない）。

実機での立ち上げで分かったこと（作業順、ファームウェア A09 が受け付けた
コマンド、ライン同期の未解決の観察、やってはいけないこと）は
[`docs/keithley2002-commissioning.md`](../../../docs/keithley2002-commissioning.md)。
