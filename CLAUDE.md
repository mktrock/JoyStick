# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト

**M5StickC Plus** 用の PlatformIO ファームウェア(Arduino フレームワーク + M5Unified)。**Keyestudio KS0008 ジョイスティックモジュール**(アナログ X/Y + 押し込みスイッチ)の値を読み、LCD とシリアルに出力する。コードはすべて `src/main.cpp` にあり、テストはない。

## コマンド

`pio` は PATH に入っていないので、フルパスで実行する(PowerShell):

```
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run                 # ビルド
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload       # 書き込み
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor      # シリアルモニタ(115200)
```

環境は `m5stick-c` の1つだけ。StickC Plus も `m5stick-c` のボード定義で動く。

## ハードウェアの制約

- 配線: X→G33、Y→G32(Grove ポート)、SW→G26(`INPUT_PULLDOWN`。このモジュールは押下で HIGH を出すので、`readSW()` で反転して CSV の sw は押下で 0 にしている)、VCC→**3V3**。
- ジョイスティックを 5V で給電しないこと。アナログ出力が 3.3V を超え、ESP32 の ADC の上限を超えてしまう。
- アナログ入力は ADC1 のピン(G32/G33/G36)だけを使うこと。ADC2 のピンは WiFi 使用中に読めない。
- G0 はブート用のピンなので使わない。
- ADC は 12bit・減衰 11dB に設定しており、値の範囲は 0–4095、中央は約 2048。中央値は起動時に校正し、BtnA を押すと校正し直す。

## 規約

- シリアル出力は `x,y,sw` の CSV で、KS0008 のサンプルスケッチと同じ形式。既存のツールで解析できるよう、この形式を変えないこと。
- LCD への描画は `M5Canvas` スプライトに描いてから `pushSprite` で送り、ちらつきを防ぐ。
