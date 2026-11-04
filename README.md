# avrhvpp

ATmega32U2 で動作する、AVR マイコン用の +12V 高電圧パラレルプログラマ（HVPP）です。
通常は STK500 互換の ISP プログラマとして動作し、専用コマンドを送ると高電圧パラレルプログラミングモード（AVRHVPP モード）に切り替わります。

ヒューズ設定の誤りで ISP が使えなくなったチップ（SPIEN 無効、DWEN 有効など）の読み出しと復旧を主な目的としています。

現在ホストツールから、シグネチャ、ヒューズ、ロックビット、ATmega328PのFlashページ／Intel HEXの読み書き、low/high/extended fuseとロックビットの書き込み、チップ消去ができます。`--read-page`、`--write-page`、`--read-hex`、`--write-hex` はATmega328Pの実機で動作確認済みです。EEPROM読み書きは未実装です。

## 対応ターゲット

現在のFlash読み書き対象はATmega328P（シグネチャ `1E 95 0F`、32 KiB、128バイト×256ページ）です。ターゲット固有のシグネチャ、ページサイズ、ページ数、ワードアドレス計算は `common/target.h` に集約しています。

次の開発目標は、3バイトのシグネチャからターゲット定義を選択し、ATmega328P以外のチップでも同じページ／Intel HEXコマンドを利用できるようにすることです。

## ディレクトリ構成

| パス | 内容 |
|---|---|
| `firmware-atmega32u2/` | プログラマ本体（ATmega32U2）のファームウェア |
| `app/` | ホスト用コマンドラインツール（`avrhvpp`） |
| `common/` | ファームウェアとホストツールで共有するコマンド定義とターゲット定義 |
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
cd firmware-atmega32u2
make            # avrhvpp.hex を生成
make write      # 外部 ISP プログラマ（avrisp, /dev/ttyACM0）で書き込み（ヒューズ hfuse=0xD9, lfuse=0x5E も設定）
make readfuse   # ATmega32U2 のヒューズを読み出し
```

`Makefile` はコンパイル時に `.d` ファイルを生成し、ヘッダの依存関係を追跡します。

### ホストツール

C++17 のコンパイラが必要です。Qt Creator で `app/avrisp.pro` を開くか、次のようにビルドします。

```sh
cd app
g++ -std=c++17 main.cpp serial.cpp -o avrhvpp
```

## 使い方

### STK500 互換モード（ISP）

電源投入時はこのモードです。ArduinoISP 由来の STK500v1 実装なので、avrdude からは `avrisp` として使えます。

```sh
avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p atmega328p -U flash:w:foo.hex
```

### AVRHVPP モード（高電圧パラレル）

```sh
./avrhvpp -p /dev/ttyACM0 --read-signature      # シグネチャを読み出し
./avrhvpp -p /dev/ttyACM0 --read-fuse-low       # low fuse を読み出し
./avrhvpp -p /dev/ttyACM0 --read-fuse-high      # high fuse を読み出し
./avrhvpp -p /dev/ttyACM0 --read-fuse-extended  # extended fuse を読み出し
./avrhvpp -p /dev/ttyACM0 --read-lock           # ロックビットを読み出し
./avrhvpp -p /dev/ttyACM0 --read-page 0 page.bin # Flashのページ0を128バイトのファイルへ保存
./avrhvpp -p /dev/ttyACM0 --read-hex flash.hex   # Flash全体をIntel HEXへ保存
./avrhvpp -p /dev/ttyACM0 --write-page 0 page.bin # 128バイトのファイルをFlashのページ0へ書き込む
./avrhvpp -p /dev/ttyACM0 --write-hex example/blink.hex # Intel HEXをFlashへ書き込む
./avrhvpp -p /dev/ttyACM0 --no-chip-erase --write-hex example/blink.hex # チップ消去せずに書き込む
./avrhvpp -p /dev/ttyACM0 --write-fuse-low 0x62 # low fuse に 0x62 を書き込む
./avrhvpp -p /dev/ttyACM0 --write-lock 0xFC     # ロックビットに 0xFC を書き込む
./avrhvpp -p /dev/ttyACM0 --chip-erase          # Flash、EEPROM、ロックビットを消去
./avrhvpp -p /dev/ttyACM0 --dump-buffer         # 共有バッファを16進ダンプする（デバッグ用）
```

ヒューズ値を誤るとISPでアクセスできなくなることがあります。ロックビットは0のビットがプログラムされ、1へ戻すにはチップ消去が必要です。チップ消去はFlashとロックビットに加え、EESAVE fuseがプログラムされていなければEEPROMも消去します。Flashページ書き込みは、ページが消去済み（全バイト `0xFF`）か、すでに入力データと完全一致する場合だけ成功します。`--write-hex` は既定で書き込み前にチップ消去します。消去したくない場合は `--no-chip-erase` を指定してください。書き込む値と対象チップを確認してから実行してください。

シグネチャの出力例:

```
0x1e, 0x95, 0x0f
```

オプション:

| オプション | 内容 |
|---|---|
| `-p <port>` | シリアルポート（Windows では `COM3` など） |
| `-s <speed>`, `--speed <speed>` | ボーレート（既定 115200。USB CDC なので通常は無関係） |
| `--debug` | 予約済みのデバッグコマンドを送る。ファームウェア側の処理は現在無効で、応答しない |
| `-rs`, `--read-signature` | シグネチャ3バイトを読み出す |
| `-rfl`, `--read-fuse-low` | low fuseを読み出す |
| `-rfh`, `--read-fuse-high` | high fuseを読み出す |
| `-rfe`, `--read-fuse-extended` | extended fuseを読み出す |
| `-rl`, `--read-lock` | ロックビットを読み出す |
| `--read-page <page> <file>` | ATmega328PのFlashページ（0～255）を128バイトのバイナリファイルへ保存する |
| `--read-hex <file>` | ATmega328PのFlash全体を読み出してIntel HEXへ保存する |
| `--write-page <page> <file>` | 128バイトのバイナリファイルをATmega328PのFlashページ（0～255）へ書き込む |
| `--write-hex <file>` | Intel HEXを解析し、チップ消去後に指定されたFlashページを書き込む |
| `--no-chip-erase` | `--write-hex` 前の自動チップ消去を無効にする。引数内の位置は問わない |
| `-wfl`, `--write-fuse-low <value>` | low fuseへ1バイト書き込む |
| `-wfh`, `--write-fuse-high <value>` | high fuseへ1バイト書き込む |
| `-wfe`, `--write-fuse-extended <value>` | extended fuseへ1バイト書き込む |
| `-wl`, `--write-lock <value>` | ロックビットへ1バイト書き込む |
| `--chip-erase` | Flash、EEPROM、ロックビットを消去する。ヒューズは変更しない |
| `--dump-buffer` | 共有バッファを16バイト×16行の16進数で表示する（デバッグ用） |

コマンドの指定は必須です。書き込み値は16進数で、`0x62`または`62`のように00～FFの範囲で指定します。複数の操作を指定した場合は左から順に実行し、失敗した時点で停止します。ポートや通信速度を含む全引数を検証してから接続するため、オプションの記述順に制約はありません。

## 通信プロトコル

### モードの切り替え

| 送信 | 応答 | 動作 |
|---|---|---|
| `.AVRHVPP.` | `AVRHVPP\r\n` | AVRHVPP モードに切り替え |
| `.BITBANG.` | `BITBANG\r\n` | BITBANG モード（ISP ピンの直接操作）に切り替え |
| 0x40 未満のバイト | — | どのモードからでも STK500 モードに戻る |

0x40 未満のバイトで自動的に戻るのは、avrdude などの STK500 クライアントを、モードを意識せずにそのまま使えるようにするためです（STK500 のコマンドは `'0'` = 0x30 などから始まります）。
その代わり、AVRHVPP モードで使えるコマンドやデータは 0x40 以上に限られます。任意の値は複数の0x40以上のバイトへ符号化して送る設計です。

ホストツールは切り替えの前に `'0'` と `' '` を送り、`0x14 0x10`（STK_INSYNC、STK_OK）が返ることを確認して同期を取ります。

### AVRHVPP モードのコマンド

コマンド値は `common/commands.h` でファームウェアとホストツールの間で共有しています。

| コマンド | 応答 | 動作 |
|---|---|---|
| `CMD_CLEAR`（0x40） | なし | 一時データとバッファアドレスを0にし、256バイトの共有バッファをクリアする |
| `CMD_DEBUG`（0x4F） | なし | 予約済み。現在は処理が無効 |
| `CMD_READ_SIGNATURE`（0x50） | 16進6文字 + `\r\n` | シグネチャ3バイトを読み出す |
| `CMD_READ_FUSE`（0x51） | 16進2文字 + `\r\n` | 共有バッファの先頭で指定したヒューズを読み出す |
| `CMD_READ_LOCK`（0x52） | 16進2文字 + `\r\n` | ロックビットを読み出す |
| `CMD_READ_PAGE`（0x53） | 成功時 `1\r\n`、失敗時 `0\r\n` | 選択したATmega328PのFlashページを共有バッファへ読み出す |
| `CMD_CHIP_ERASE`（0x60） | 成功時 `1\r\n`、失敗時 `0\r\n` | チップ消去を実行する |
| `CMD_WRITE_FUSE`（0x61） | 成功時 `1\r\n`、失敗時 `0\r\n` | 共有バッファで指定したlow/high/extended fuseへ値を書き込む |
| `CMD_WRITE_LOCK`（0x62） | 成功時 `1\r\n`、失敗時 `0\r\n` | 共有バッファで指定した値をロックビットへ書き込む |
| `CMD_WRITE_PAGE`（0x63） | 成功時 `1\r\n`、失敗時 `0\r\n` | 共有バッファの128バイトを選択したATmega328PのFlashページへ書き込む |
| `CMD_ADDR0` / `CMD_ADDR1`（0x70 / 0x71） | なし | 共有バッファの読み書き位置を設定する |
| `CMD_DATA0` / `CMD_DATA1`（0x72 / 0x73） | なし | 共有バッファの現在位置へ1バイト書き込み、位置を1進める |
| `CMD_LATCH_PAGE_NUM`（0x7E） | なし | 共有バッファ先頭の2バイトをリトルエンディアンのページ番号として確定し、共有バッファをクリアする |
| `CMD_READ_16BYTES`（0x7F） | 16進32文字 + `\r\n` | 共有バッファの現在位置から16バイト読み出し、位置を16進める |
| `CMD_LOAD_7BITS`（0x80～0xFF） | なし | 下位7ビットを一時データとしてロードする |

ファームウェアのSTK500処理とAVRHVPP処理は、同じ256バイトの `flush_buffer` を共有します。AVRHVPPの操作では、最初に `CMD_CLEAR` でバッファと書き込み位置を初期化します。任意の1バイトは `CMD_LOAD_7BITS` で下位7ビットをロードした後、最上位ビットに応じて `CMD_DATA0` または `CMD_DATA1` を送ることで現在位置へ格納されます。格納後、位置は自動的に1進みます。

`CMD_READ_FUSE` の対象番号は0がlow fuse、1がhigh fuse、2がextended fuseです。ホストツールは対象番号を共有バッファの0番へ格納してから読み出しコマンドを送ります。

ヒューズ書き込みでは共有バッファの0番に対象番号、1番に値を格納してから `CMD_WRITE_FUSE` を送ります。ロックビット書き込みでは0番に値を格納してから `CMD_WRITE_LOCK` を送ります。現在のファームウェアは値の設定済み状態を別途管理せず、共有バッファの内容をそのまま使用します。

Flashページ番号は16ビットです。`CMD_CLEAR` 後、下位バイト、上位バイトの順に `CMD_DATA0` / `CMD_DATA1` で共有バッファへ格納し、`CMD_LATCH_PAGE_NUM` で確定します。ラッチ時に共有バッファと書き込み位置は初期化されます。`CMD_READ_PAGE` が成功すると128バイトが共有バッファへ格納されるため、バッファアドレスを0に設定し、`CMD_READ_16BYTES` を8回送って回収します。読み出し時はシグネチャ3バイトがATmega328Pの `1E 95 0F` と一致することも確認します。ページサイズ、ページ数、シグネチャは `common/target.h` で共有しています。

`--read-hex` は `CMD_READ_PAGE` で全256ページを読み出し、すべて成功した後に出力ファイルを作成します。Intel HEXは16バイトのデータレコードとEOFレコードで構成し、全バイト `0xFF` のデータレコードは省略します。省略されたアドレスは消去済みの `0xFF` として扱います。

Flashページ書き込みでは、上記の手順でページ番号をラッチした後、`CMD_DATA0` / `CMD_DATA1` で128バイトを先頭から連続して共有バッファへ格納し、`CMD_WRITE_PAGE` を送ります。ファームウェアはページ指定済み、受信サイズ、ATmega328Pの完全なシグネチャを確認します。現在のページが入力と完全一致する場合は再試行として成功し、全バイト `0xFF` の場合だけ実際に書き込みます。書き込み後はターゲットから全128バイトを再読出しし、完全一致した場合だけ成功を返します。

`--write-hex` はIntel HEXのチェックサム、EOF、重複データ、ATmega328Pの32 KiB範囲を接続前に検証します。データ、EOF、拡張セグメントアドレス、拡張リニアアドレス、開始アドレスの各レコードに対応します。検証成功後、既定ではチップ消去を実行し、HEXで指定されたページを読み出して消去を確認してから `CMD_WRITE_PAGE` で書き込みます。HEXで省略された領域は書き込み対象になりません。

`--no-chip-erase` を指定すると自動チップ消去を省略します。この場合はHEXで指定された全ページを先に読み出し、各ページが消去済みまたは入力イメージと完全一致することを確認してから、必要なページだけを書き込みます。既存内容と競合するページがあれば、1ページも書き込まず終了します。

実装済みのシグネチャ、ヒューズ、ロックビット、Flashページの読み出し／書き込みとチップ消去は、毎回プログラミングモードに入り、処理後に抜けます。モードに入れなかった場合の応答は `0\r\n` です。
プログラミングモードに入れたかどうかは、RDY/BSY が High になることと、シグネチャの先頭がメーカーコード 0x1E であることで判定しています。

## 制約と今後の課題

- Flash読み書きのターゲットは現在ATmega328Pに固定されています。別チップへ対応するときは、シグネチャ、Flash容量、ページサイズ、ページ数、HVPPのアドレス指定と書き込み手順をデータシートで確認し、ターゲット定義として追加します。
- ページ番号は16ビットで送信します。実際に有効な範囲は選択されたターゲット定義のページ数によって制限されます。
- `CMD_DEBUG`とEEPROMの読み書きは未実装です。
- 0x40未満でSTK500モードへ戻る仕様を維持するため、任意の8ビット値は `CMD_LOAD_7BITS` と `CMD_DATA0` / `CMD_DATA1` などを使って0x40以上のバイトへ符号化します。
- プログラミングモードへは、VCC を常時供給したまま XTAL1 を 6 回トグルし、+12V を印加する方式で入ります。RSTDISBL がプログラムされたチップでは、ターゲットのプログラムが動いたままになり、モードに入れない可能性があります。対応するには VCC の切り替え回路と、より速い +12V のスイッチが必要です。

## ライセンス

Copyright (C) 2022 S.Fuchita (@soramimi_jp)
