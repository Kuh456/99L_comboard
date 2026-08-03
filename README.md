# 99L 通信基板

ESP32-S3上でLoRa地上局コマンドをCANへ中継し、CANテレメトリとGNSSを39 byteの
`Payload`にまとめてLoRa送信し、同じテレメトリをSDへCSV記録する通信基板です。

## 状態の決定

シーケンスおよび制御基板の実状態は、CAN ID `0x200` の制御基板ステータスからのみ
決定します。LoRaコマンドは要求、CAN送信成功はバスへ送れたことを示すだけです。
いずれも実状態や`Payload.status`、シーケンス連動のGNSS/SD状態を直接変更しません。

## CAN protocol

CAN 2.0の11-bit標準IDを使用し、通信速度は125 kbpsです。拡張IDのフレーム、未定義ID、
および表と異なるDLCは受理しません。コマンドは通信基板から各制御基板へ送信し、
テレメトリとステータスは各制御基板から通信基板が受信します。

### 送信コマンド

すべてDLC 1で、data[0]にはLoRaコマンドと同じASCII byteを格納します。残りのbyteは
CANフレームへ含めません。`g` / `h`は通信基板内のGNSS操作専用で、CANへは送信しません。

|  CAN ID |      data[0] | コマンド                 |
| ------: | -----------: | ------------------------ |
| `0x001` | `0x45` (`E`) | fin control stop         |
| `0x003` | `0x7a` (`z`) | parachute emergency stop |
| `0x005` | `0x73` (`s`) | sequence start           |
| `0x00a` | `0x71` (`q`) | sequence stop            |
| `0x011` | `0x6c` (`l`) | logging start            |
| `0x01e` | `0x6d` (`m`) | logging stop             |
| `0x30d` | `0x6f` (`o`) | parachute open           |
| `0x30e` | `0x63` (`c`) | parachute close          |

CANのIDが小さいほどバス調停上の優先度は高くなります。`0x30d` / `0x30e`は現在の
指定値であり、緊急停止`0x003`より低い優先度です。

### 受信テレメトリ

|  CAN ID | DLC | data形式                                       | 通信基板での扱い                             |
| ------: | --: | ---------------------------------------------- | -------------------------------------------- |
| `0x10a` |   3 | `[pressure0, pressure1, pressure2]` (`u8 × 3`) | Payload `air_pressure[0..3]`へコピー         |
| `0x11a` |   6 | `X, Y, Z: i16`                                 | Payload `acceleration`へコピー               |
| `0x120` |   6 | `X, Y, Z: i16`                                 | Payload `angle_speed`へコピー                |
| `0x13a` |   6 | `X, Y, Z: i16`                                 | Payloadとの対応が未定義のため破棄数のみ記録  |
| `0x14a` |   6 | `X, Y, Z: i16`                                 | Payload `integrated_angle`へコピー           |
| `0x200` |   1 | status bitfield                                | 制御基板の実状態およびPayload `status`へ反映 |

3軸の`i16`値は符号付きbig endianです。data[0..2]がX、data[2..4]がY、
data[4..6]がZで、各軸は上位byte、下位byteの順です。例として
`[0x00, 0x01, 0xff, 0xff, 0x7f, 0xff]`は`[1, -1, 32767]`になります。
AirPressureの3 byteは数値へ結合せず、そのままPayloadへ保持します。物理単位、倍率、
送信周期は現時点のコードでは定義していません。

### CAN ID 0x200

制御基板から通信基板への標準ID、DLC 1の受信専用メッセージです。1 byteをphaseと
flagsに分割する根拠は確認できておらず、現在確認できる仕様は次のbitfieldだけです。
通信基板の送信型にはこのIDが存在しないため、復旧probeとして送ることもできません。

| bit | 意味            | 1の意味                |
| --: | --------------- | ---------------------- |
|   0 | TOP             | 検出済み               |
|   1 | main power      | ON                     |
|   2 | emergency power | ON                     |
|   3 | control active  | active                 |
|   4 | reserved        | 未知bitとしてrawに保持 |
|   5 | sequence active | active                 |
|   6 | liftoff         | 検出済み               |
|   7 | parachute motor | open                   |

byte内はLSBをbit 0とします。複数byte値はありません。独立したphase値、fault flags、
送信周期、起動時の送信値、状態ラッチ仕様は未確定です。未知bitはrawに保持しつつ、
既知bitは常に解釈します。

`ControllerLinkState`は起動時`Unknown`、有効な受信後`Online`です。周期が未確定のため
実機のstale timeoutは現在無効です。純粋判定関数は設定可能ですが、推測した時間を
組込み設定へ入れていません。周期確認後、周期の3倍等の根拠ある値を設定してください。
CAN controllerのTEC/REC/Bus Off healthとは別の状態です。

## LoRaコマンド

UARTアップリンクは固定3 byteフレームです。旧1 byteコマンドは受理しません。

```text
[0] Header   = 0x55
[1] Command
[2] Checksum = Header ^ Command
```

Headerまたはchecksumが一致しないフレームは破棄します。UARTの分割受信と連続フレームに
対応し、正常なフレームのCommandだけを次の表に従って処理します。

| byte      | 要求                             |
| --------- | -------------------------------- |
| `s` / `q` | sequence start / stop            |
| `z`       | parachute emergency stop         |
| `l` / `m` | CAN logging start / stop command |
| `E`       | fin control stop                 |
| `o` / `c` | parachute open / close           |
| `g` / `h` | GNSS manual on / off             |

未知byteは無視します。`s`、`q`はqueued → transmitted → confirmed/failedへ進み、
confirmedはそれぞれ0x200のsequence bit ON、OFFで成立します。要求時点ですでに目標bitなら
CAN送信せず`AlreadySatisfied`とします。`z`は送信済みまで管理しますが、0x200に明示ACKや
緊急停止状態bitがないためCompletedにはしません。特にliftoff=0は起動直後にも成立するため
完了根拠に使用しません。

確認timeoutは暫定500 msで、これは実行失敗ではなく`ConfirmationTimedOut`、すなわち
確認不能を意味します。実状態は最後の0x200のままです。0x200周期を実機測定後に再設定が
必要です。自動再送は冪等性未確認のため行いません。
同じカテゴリの新しい要求は古いqueued要求をsupersededにします。対象はsequence start/stop、
logging start/stop、parachute open/closeです。復旧後は各カテゴリの最新だけを送り、緊急停止は
専用Signalから通常FIFOより優先します。ただし一度失敗した緊急停止の自動再送はしません。

## Payload

全39 byte、little endianです。offset 0..2はLoRa address/channel、3は`0xaa`、4は
制御基板0x200のraw status、5..8 latitude i32、9..12 longitude i32、13..14 height
i16、15..20 gyro、21..26 acceleration、27..32 integrated angle、33..35 pressure、
36 air speed、37 fin angle、38 checksumです。checksumはoffset 3..37のXORです。
形式、順序、checksumは変更していません。

0x110/0x12aの旧LiftOff/TOPは値の送信側仕様が未確認で、状態の唯一性を守るためPayloadへ
反映せず受信数だけ記録します。TOP即時送信は0x200 bit 0の立上りで行い、初回受信時に
すでにTOP=1でも送信します。同じTOP=1の周期フレームでは再トリガーしません。
FinAngleはCAN `[i16;3]` とPayload `i8`の対応が未定義なので変換せず破棄数を記録します。
Payloadの0は実測0と未更新を区別できません。

## GNSS・SD・故障時挙動

StartSequence要求の受理時にSDの`logging_requested`を立て、GNSSをON要求しますが、
Payloadのsequence bitは変更しません。StopSequence確認だけでは停止せず、独立した
StartLogging/StopLoggingコマンドをログ開始・停止の基準にします。SDは
`logging_active`を分離し、open/write/flush失敗時はactiveを解除してエラー・drop数を
記録します。CSV列は変更していません。GNSSは設定が全て成功してからON扱いにし、Invalid
fixを採用せず、overflowしたNMEA行全体を次の改行まで破棄します。

CAN Bus OffではTWAIを再起動し、TEC/RECと連続エラーを別に記録します。LoRa UARTの
partial write、write/flush/read errorを処理し、AUXがHighでない場合も最大1000 msで
復帰します。このAUX値は現行E220設定での実測確認が必要です。SD、GNSS、LoRaの各失敗は
カウンタへ残り、asyncタスクをpanicまたは永久停止させません。

## LoRa折り返し送信側試験

`test/lora_roundtrip_tx.rs`はESP32-S3とE220-900T22S用の独立したハードウェア試験です。
本番と同じUART2（TX GPIO11、RX GPIO12、115200 baud）、AUX GPIO8、M0 GPIO9、
M1 GPIO10、39 byte Payload、宛先`00 00`、channel `04`を使用します。M0/M1はLowです。
E220の固定／透過送信設定は不揮発設定に依存し、このプログラムからは変更しません。

ダウンリンクではPayloadのstatus byteを8-bitの`downlink_sequence`として使用します。
折り返し側から返す試験専用3 byteアプリケーションデータは次の形式です。本番用の
`55 command (55 ^ command)`形式は変更していません。

```text
[0] command（s/q/z/l/m/E/o/c/g/h）
[1] downlink_sequence
[2] command ^ downlink_sequence
```

受信側はダウンリンクPayloadのoffset 4からsequenceを取り出し、`t_wait`後に上記3 byteを
返してください。受信側MCUから固定送信する際に必要な宛先・channel 3 byteはE220への
UART書き込み時だけ先頭へ追加し、無線上のアプリケーションデータはこの3 byteとします。
送信側では1 byte目を受けてから20 ms以内に3 byteが揃わない応答をlength errorとして
破棄します。

ESP32-S3への書き込みとモニタは次のコマンドで行います。`hardware-test` featureを
明示した場合だけ試験ファームウェアをビルドし、`.cargo/config.toml`のrunnerが
`espflash`による書き込みとシリアルモニタを実行します。

```console
cargo test --test lora_roundtrip_tx --features hardware-test
```

実機へ書き込まず、コンパイルだけを確認する場合は`--no-run`を付けます。

```console
cargo test --test lora_roundtrip_tx --features hardware-test --no-run
```

各試行は`TEST,`、100周期ごとの集計は`SUMMARY,`で始まります。`success_bp`は成功率を
basis point（10000 = 100%）で表します。CSVログ出力自体による遅延も
`schedule_lateness_us`へ現れるため、測定時はUSBシリアルの受信を継続してください。
