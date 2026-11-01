# avrpara

ATmega32U2 で動作する、AVR マイコン用の +12V 高電圧パラレルプログラマ（HVPP）です。
通常は STK500 互換の ISP プログラマとして動作し、専用コマンドを送ると高電圧パラレルプログラミングモード（AVRPARA モード）に切り替わります。

ヒューズ設定の誤りで ISP が使えなくなったチップ（SPIEN 無効、DWEN 有効など）の読み出しと復旧を主な目的としています。

> **開発中です。** 現在ホストツールから、シグネチャ、ヒューズ、ロックビットの読み出し、low/high/extended fuseとロックビットの書き込み、チップ消去ができます。

## ディレクトリ構成

| パス | 内容 |
|---|---|
| `atmega32u2-firmware/` | プログラマ本体（ATmega32U2）のファームウェア |
| `app/` | ホスト用コマンドラインツール（`avrisp`） |
| `common/` | ファームウェアとホストツールで共有するコマンド定義 |
| `pcb/` | KiCad の回路図・基板データ、ガーバー |

## ハードウェア

- MCU: ATmega32U2（16MHz クリスタル、USB CDC で接続）
- ターゲット: 28 ピン ZIF ソケット（ATmega328P で動作確認済み）
- +12V: MT3608 昇圧モジュール（J4 経由）→ 12V レギュレータ（U3）で生成し、PC817 フォトカプラでターゲットの RESET に印加します。印加しないときは R6 により RESET が GND に落ちるので、ソケットのチップはリセット状態で停止しています。
- ターゲットの VCC は常時供給されます（VCC の切り替え回路はありません）。

### ピン割り当て（ATmega32U2 ↔ ターゲット）

| ATmega32U2 | 信号 | ターゲット（ATmega328P DIP28） |
|---|---|---|
| PD0–PD7 | DATA0–7 | PB0–PB5（14–19 番）、PC0–PC1（23、24 番） |
| PB3 | RDY/BSY | PD1（3 番） |
| PB4 | PAGEL | PD7（13 番） |
| PB5 | XA1 | PD6（12 番） |
| PB6 | XA0 | PD5（11 番） |
| PB7 | XTAL1 | XTAL1（9 番） |
| PC2 | +12V 制御（PC817） | RESET（1 番） |
| PC4 | OE | PD2（4 番） |
| PC5 | WR | PD3（5 番） |
| PC6 | BS1 | PD4（6 番） |
| PC7 | BS2 | PC2（25 番） |

STK500 モードでは PB0（RST）、PB1（SCK）、PB2（MOSI）、PB3（MISO）を ISP 用に使い、6 ピンヘッダ J2 に出しています。PB3 は RDY/BSY と共用です。
J3 をショートすると J2 の RST が ATmega32U2 自身の RESET につながり、外部のプログラマから ATmega32U2 にファームウェアを書き込めます。

## ビルドと書き込み

### ファームウェア

avr-gcc と avrdude が必要です。

```sh
cd atmega32u2-firmware
make            # main.hex を生成
make write      # 外部 ISP プログラマ（avrisp, /dev/ttyACM0）で書き込み（ヒューズ hfuse=0xD9, lfuse=0x5E も設定）
make readfuse   # ATmega32U2 のヒューズを読み出し
```

`Makefile` にはヘッダの依存関係が書かれていません。ヘッダを変更したときは `make clean` してからビルドしてください。

### ホストツール

C++17 のコンパイラが必要です。Qt Creator で `app/avrisp.pro` を開くか、次のようにビルドします。

```sh
cd app
g++ -std=c++17 main.cpp serial.cpp -o avrisp
```

## 使い方

### STK500 互換モード（ISP）

電源投入時はこのモードです。ArduinoISP 由来の STK500v1 実装なので、avrdude からは `avrisp` として使えます。

```sh
avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p atmega328p -U flash:w:foo.hex
```

### AVRPARA モード（高電圧パラレル）

```sh
./avrisp -p /dev/ttyACM0 --debug               # シグネチャ、ヒューズ、ロックビットをまとめて読み出し
./avrisp -p /dev/ttyACM0 --read-signature      # シグネチャを読み出し
./avrisp -p /dev/ttyACM0 --read-fuse-low       # low fuse を読み出し
./avrisp -p /dev/ttyACM0 --read-fuse-high      # high fuse を読み出し
./avrisp -p /dev/ttyACM0 --read-fuse-extended       # extended fuse を読み出し
./avrisp -p /dev/ttyACM0 --read-lock           # ロックビットを読み出し
./avrisp -p /dev/ttyACM0 --write-fuse-low 0x62 # low fuse に 0x62 を書き込む
./avrisp -p /dev/ttyACM0 --write-lock 0xFC      # ロックビットに 0xFC を書き込む
./avrisp -p /dev/ttyACM0 --chip-erase           # Flash、EEPROM、ロックビットを消去
```

ヒューズ値を誤るとISPでアクセスできなくなることがあります。ロックビットは0のビットがプログラムされ、1へ戻すにはチップ消去が必要です。チップ消去はFlashとロックビットに加え、EESAVE fuseがプログラムされていなければEEPROMも消去します。書き込む値と対象チップを確認してから実行してください。

出力例:

```
0x1e, 0x95, 0x0f, 0x62, 0xd9, 0xff, 0xff
```

左から順に、シグネチャ 3 バイト、low fuse、high fuse、extended fuse、ロックビットです。

オプション:

| オプション | 内容 |
|---|---|
| `-p <port>` | シリアルポート（Windows では `COM3` など） |
| `-s <speed>`, `--speed <speed>` | ボーレート（既定 115200。USB CDC なので通常は無関係） |
| `--debug` | シグネチャ3バイト、low/high/extended fuse、ロックビットをまとめて読み出す |
| `-rs`, `--read-signature` | シグネチャ3バイトを読み出す |
| `-rfl`, `--read-fuse-low` | low fuseを読み出す |
| `-rfh`, `--read-fuse-high` | high fuseを読み出す |
| `-rfe`, `--read-fuse-extended` | extended fuseを読み出す |
| `-rl`, `--read-lock` | ロックビットを読み出す |
| `-wfl`, `--write-fuse-low <value>` | low fuseへ1バイト書き込む |
| `-wfh`, `--write-fuse-high <value>` | high fuseへ1バイト書き込む |
| `-wfe`, `--write-fuse-extended <value>` | extended fuseへ1バイト書き込む |
| `-wl`, `--write-lock <value>` | ロックビットへ1バイト書き込む |
| `--chip-erase` | Flash、EEPROM、ロックビットを消去する。ヒューズは変更しない |

コマンドの指定は必須です。書き込み値は16進数で、`0x62`または`62`のように00～FFの範囲で指定します。複数の操作を指定した場合は左から順に実行し、失敗した時点で停止します。ポートや通信速度を含む全引数を検証してから接続するため、オプションの記述順に制約はありません。

## 通信プロトコル

### モードの切り替え

| 送信 | 応答 | 動作 |
|---|---|---|
| `.AVRPARA.` | `AVRPARA\r\n` | AVRPARA モードに切り替え |
| `.BITBANG.` | `BITBANG\r\n` | BITBANG モード（ISP ピンの直接操作）に切り替え |
| 0x40 未満のバイト | — | どのモードからでも STK500 モードに戻る |

0x40 未満のバイトで自動的に戻るのは、avrdude などの STK500 クライアントを、モードを意識せずにそのまま使えるようにするためです（STK500 のコマンドは `'0'` = 0x30 などから始まります）。
その代わり、AVRPARA モードで使えるコマンドやデータは 0x40 以上に限られます。任意の値は複数の0x40以上のバイトへ符号化して送る設計です。

ホストツールは切り替えの前に `'0'` と `' '` を送り、`0x14 0x10`（STK_INSYNC、STK_OK）が返ることを確認して同期を取ります。

### AVRPARA モードのコマンド

コマンド値は `common/commands.h` でファームウェアとホストツールの間で共有しています。

| コマンド | 応答 | 動作 |
|---|---|---|
| `CMD_DEBUG`（0x40） | 16進14文字 + `\r\n` | シグネチャ3バイト、low/high/extended fuse、ロックビットを読み出す |
| `CMD_READ_SIGNATURE`（0x41） | 16進6文字 + `\r\n` | シグネチャ3バイトを読み出す |
| `CMD_CHIP_ERASE`（0x4F） | 成功時 `1\r\n`、失敗時 `0\r\n` | チップ消去を実行する |
| `CMD_READ_FUSE`（0x50） | 16進2文字 + `\r\n` | 直前にロードした番号のヒューズを読み出す |
| `CMD_READ_LOCK`（0x5F） | 16進2文字 + `\r\n` | ロックビットを読み出す |
| `CMD_WRITE_FUSE`（0x60） | 成功時 `1\r\n`、失敗時 `0\r\n` | 選択したlow/high/extended fuseへ値を書き込む |
| `CMD_WRITE_LOCK`（0x6F） | 成功時 `1\r\n`、失敗時 `0\r\n` | ロックビットへ値を書き込む |
| `CMD_ADDR0`～`CMD_DATA1`（0x70～0x73） | なし | 書き込み対象番号と値の最上位ビットを確定する |
| `CMD_LOAD_7BITS`（0x80～0xFF） | なし | 下位7ビットを一時データとしてロードする |

`CMD_READ_FUSE` の対象番号は0がlow fuse、1がhigh fuse、2がextended fuseです。ホストツールは `CMD_LOAD_7BITS | 対象番号` と `CMD_ADDR0` / `CMD_ADDR1` を送ってから読み出しコマンドを送ります。

書き込みでは、対象番号を `CMD_LOAD_7BITS` と `CMD_ADDR0` / `CMD_ADDR1` で指定し、値を `CMD_LOAD_7BITS` と `CMD_DATA0` / `CMD_DATA1` で設定してから `CMD_WRITE_FUSE` を送ります。対象番号は0～2だけが有効です。

ロックビット書き込みも同じ方法で値を設定してから `CMD_WRITE_LOCK` を送ります。ファームウェアは対象と値が設定されていない書き込み要求を拒否します。

各読み出し／書き込みコマンドは毎回プログラミングモードに入り、処理後に抜けます。モードに入れなかった場合の応答は `0\r\n` です。
プログラミングモードに入れたかどうかは、RDY/BSY が High になることと、シグネチャの先頭がメーカーコード 0x1E であることで判定しています。

## 制約と今後の課題

- フラッシュとEEPROMの個別の読み書きは未実装です。
- 0x40未満でSTK500モードへ戻る仕様を維持するため、任意の8ビット値は `CMD_LOAD_7BITS` と `CMD_DATA0` / `CMD_DATA1` などを使って0x40以上のバイトへ符号化します。
- プログラミングモードへは、VCC を常時供給したまま XTAL1 を 6 回トグルし、+12V を印加する方式で入ります。RSTDISBL がプログラムされたチップでは、ターゲットのプログラムが動いたままになり、モードに入れない可能性があります。対応するには VCC の切り替え回路と、より速い +12V のスイッチが必要です。

## ライセンス

Copyright (C) 2022 S.Fuchita (@soramimi_jp)
