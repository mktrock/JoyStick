pyserial 3.5 (https://github.com/pyserial/pyserial, BSD-3-Clause) を同梱しています。
Fusion 内蔵の Python には pyserial が入っていないためです。

BLE 接続用に bleak 3.0.2 (https://github.com/hbldh/bleak, MIT) と、その依存パッケージ
(winrt-runtime / winrt-Windows.* 3.2.1、typing_extensions)も同梱しています。
winrt は Fusion の Python 3.14 用(cp314, win_amd64)のバイナリです。Fusion の Python の
バージョンが変わったら、次のコマンドで入れ直してください(serial/ と README.md 以外を消してから実行):

```
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m pip install --target fusion/JoyOrbit/lib --python-version 3.14 --platform win_amd64 --only-binary=:all: bleak
```
