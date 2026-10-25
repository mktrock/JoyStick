# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト

**Seeed Studio XIAO ESP32C3** 用の PlatformIO ファームウェア(Arduino フレームワーク)。**Keyestudio KS0008 ジョイスティックモジュール**(アナログ X/Y + 押し込みスイッチ)の値を読み、USB シリアルと BLE に出力する。コードはすべて `src/main.cpp` にあり、テストはない。

## コマンド

`pio` は PATH に入っていないので、フルパスで実行する(PowerShell):

```
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run                                         # ビルド(全環境)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e xiao-esp32c3 -t upload --upload-port COM8   # 書き込み
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor -e xiao-esp32c3 -p COM8      # シリアルモニタ(115200)
```

- 環境は `xiao-esp32c3`(本番。発熱を抑えるため BLE・WiFi は止めてあり、USB シリアルのみに出力する)、`xiao-esp32c3-ble`(`-DENABLE_BLE`。BLE でも出力する)、`xiao-esp32c3-sim`(`-DSIMULATE`。配線なしで円を描く擬似入力を出し、BOOT ボタンを SW として使う)。
- ポートは必ず指定する。この PC には Bluetooth の仮想 COM ポートがあり、自動検出だとそちらを選んで書き込みに失敗する。XIAO は USB Serial/JTAG(VID:PID=303A:1001)で、番号は `pio device list` で確認する。

## Fusion アドイン

`fusion/JoyOrbit/` はシリアルの `x,y,sw` を読んで Fusion のビューを動かすアドイン。SW の短押し(離したとき)でオービット ⇔ パンとモードが切り替わり、長押しでホームビューに戻る。押下中はビューを動かさない(モードはアドインが持ち、`M:<mode>\n` を本体に送る。本体は表示器がないので `# mode: <mode>` をシリアルに返すだけ)。Fusion 内蔵 Python には pyserial がないので `lib/` に同梱している。USB のポートが見つからないときは BLE で接続する(本体は Nordic UART Service をデバイス名 `JoyStick-XIAO` で公開し、USB と同じ行を TX の notify で送り、`M:` は RX への write で受ける。アドインは名前か NUS の UUID で見つける)。BLE 用の bleak と winrt も `lib/` に同梱している。winrt は Fusion の Python 3.14 用のバイナリなので、Fusion の Python が上がったら入れ直す(手順は `lib/README.md`)。シリアル読み取りは別スレッドで行い、カメラ操作は `fireCustomEvent` でメインスレッドに渡して実行する(Fusion API はメインスレッド以外から呼べない)。

## ハードウェアの制約

- 配線: Y→D1(GPIO3)、X→D2(GPIO4)、SW→D3(GPIO5、`INPUT_PULLDOWN`。このモジュールは押下で HIGH を出すので、`readSW()` で反転して CSV の sw は押下で 0 にしている)、VCC→**3V3**。配線図は `docs/wiring.svg`。
- ジョイスティックを 5V で給電しないこと。アナログ出力が 3.3V を超え、ESP32-C3 の ADC の上限を超えてしまう。
- アナログ入力は ADC1 のピン(GPIO0–4)だけを使うこと。ADC2(GPIO5)は WiFi/BLE 使用中に安定しない。
- GPIO2・GPIO8・GPIO9 はストラッピングピンなので配線に使わない(GPIO9 は基板の BOOT ボタンで、起動後は校正ボタンとして読んでいる)。
- ADC は 12bit・減衰 11dB に設定しており、値の範囲は 0–4095。中央値は起動時に校正し、BOOT ボタンを押すと校正し直す。

## 規約

- シリアル出力は `x,y,sw` の CSV で、KS0008 のサンプルスケッチと同じ形式。既存のツールで解析できるよう、この形式を変えないこと。BLE の notify でも同じ形式で送る(出力は `emit()` を通す)。
