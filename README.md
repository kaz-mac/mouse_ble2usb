# M5Stack AtomS3U BLE Mouse to USB HID Bridge

M5Stack AtomS3U を BLE マウス用の USB HID 変換アダプタとして使う Arduino スケッチです。

PC 側には通常の USB マウスとして認識されます。PC に Bluetooth 機能が無い場合でも、AtomS3U と BLE マウスをペアリングすることで BLE マウスを使えるようにします。

※このREADMEはAIが90%くらい書いています

## 主な機能

- BLE HID マウスをスキャンして接続
- USB HID マウスとして PC へ転送
- 対応するボタン
    - 左クリック、右クリック、中クリック、戻る、進む
    - ホイールスクロール
    - カーソル移動
- HID Report Map を解析して、複数の BLE マウス形式に対応（したつもり）
- AtomS3U のボタンで接続/切断
- 内蔵 RGB LED で状態表示
- 外部 UART へデバッグログ出力

## 対応ハードウェア

- M5Stack AtomS3U
- BLE HID マウス
- USB HID マウスを認識できる PC

USB-UART アダプターを使用すればデバッグログを見ることもできます。

## ピン設定

デバッグログを見たい場合は AtomS3U の以下のポートに接続します。

- `G17`: UART TX、USB-UART 変換器の RX へ接続
- `G14`: UART RX、USB-UART 変換器の TX へ接続
- `GND`: USB-UART 変換器の GND と共通
- ボーレート: `115200`

ログを見るだけなら、最低限 `G17` と `GND` のみでOK。

## Arduino IDE でのコンパイル

### 1. 追加ライブラリ

ライブラリマネージャから追加インストールが必要なライブラリはありません。

以下は ESP32 Arduino Core に含まれる標準ライブラリを使用しています。

- `BLEDevice`
- `BLEClient`
- `BLESecurity`
- `USB`
- `USBHIDMouse`

### 2. ボード設定

Arduino IDE で AtomS3U 相当の ESP32-S3 ボードを選択します。

推奨:

- Board: `M5AtomS3`
- Port: AtomS3Uが接続されているポート（UARTポートではない点に注意）
- USB Mode: `USB-OTG (TinyUSB)` または TinyUSB HID が有効になる設定
- USB CDC On Boot: 任意。ただしこのスケッチはログを外部 UART に出します

ボードメニュー名は Arduino-ESP32 Core のバージョンや導入環境により異なります。

### 4. 書き込み時の注意

このスケッチは起動後に USB HID マウスとして動作します。そのため、環境によっては次回書き込み時に通常のシリアルポートとして認識されにくいことがあります。

書き込みに失敗する場合は、AtomS3U をダウンロードモードに入れてから書き込んでください。

1. AtomS3U の再度ボタンを3秒以上押し続ける（隙間から緑LEDが見えたら離す）
2. Arduino IDE のポートを選び直す（変わってしまった場合）
3. 書き込む

## 使い方

1. スケッチを書き込んだ AtomS3U を PC の USB ポートへ接続します。（LED赤）
2. BLE マウスをペアリングモードにします。
3. AtomS3U のボタンを押します。（LED青点滅）
4. BLE マウスが見つかると接続し、PC からは USB マウスとして使えるようになります。（LED緑）
5. 接続中に AtomS3U のボタンを押すと BLE マウスを切断します。（LED赤）
6. 切断後にもう一度ボタンを押すと、再びスキャンします。（LED青点滅）

スキャン時間内にマウスが見つからない場合は待機状態に戻ります。再度ボタンを押すとスキャンできます。

## LED 表示

AtomS3U の内蔵 RGB LED で状態を表示します。

- 赤点灯: 待機中
- 青点滅: BLE マウスをスキャン中
- 黄点灯: 接続処理中
- 緑点灯: BLE マウス接続済み
- 赤点滅: エラー状態

## デバッグログ

外部 UART にログを出力します。

詳細な BLE レポートログはスケッチ先頭の `DEBUG_LOG` で切り替えできます。

```cpp
#define DEBUG_LOG 0
```

`1` にすると、BLE レポートの生データや HID Report Map 解析結果、USB へ送る値などを出力します。調査時に有効にしてください。

## 技術的解説

### 全体構成

AtomS3U は BLE 側では HID Host 相当のクライアントとして動作し、USB 側では HID Mouse デバイスとして動作します。

```text
BLE Mouse  -- BLE HID -->  AtomS3U  -- USB HID -->  PC
```

PC から見ると AtomS3U は通常の USB マウスです。BLE マウスそのものは PC に直接接続されません。

### BLE HID の処理

BLE HID マウスは HID Service `0x1812` を広告します。スケッチはボタン押下後にこの service を持つ BLE デバイスをスキャンし、見つかったデバイスへ接続します。

接続後は以下を行います。

- HID Protocol Mode を Report Protocol に設定
- HID Report Map `0x2A4B` を読み取り
- notify / indicate 可能な characteristic を購読
- CCCD `0x2902` を明示的に有効化
- Report Reference `0x2908` から Report ID と Report Type を記録

### HID Report Map 解析

BLE マウスの入力レポート形式は機種ごとに異なります。

たとえば、あるマウスでは `buttons, dx8, dy8, wheel8` のような単純な形式ですが、別のマウスでは以下のように分離されることがあります。

- Report ID 1: ボタンとホイール
- Report ID 2: X/Y 移動
- Report ID 18: 拡張ボタンや独自機能

そのため、固定のバイト位置で解釈するとマウスによって誤動作します。このスケッチでは HID Report Map を解析し、以下の usage をビット単位で取り出します。

- Button 1: 左クリック
- Button 2: 右クリック
- Button 3: 中クリック
- Button 4: 戻る
- Button 5: 進む
- Generic Desktop X
- Generic Desktop Y
- Generic Desktop Wheel

Report Map が読めない、または対応外の構造だった場合は、既知の一般的なマウスレポート形式を fallback として解釈します。

### USB HID への転送

解析した BLE 入力は `USBHIDMouse` を使って PC へ送信します。

- ボタン: `Mouse.press()` / `Mouse.release()`
- 移動: `Mouse.move(dx, dy, wheel)`
- 対応ボタン: 左、右、中、戻る、進む

USB HID の相対移動量は `int8_t` なので、BLE 側が 16bit 移動量を送ってきた場合は `-127` から `127` に丸めて転送します。

### 再ペアリング時の状態リセット

マウスを切り替える場合、BLE クライアント内部の GATT service / characteristic キャッシュを引きずると誤動作することがあります。

そのため、切断時と次回接続前に以下を初期化します。

- BLEClient の破棄と再作成
- レポートキュー
- HID Report Reference 情報
- HID Report Map 解析結果
- USB ボタン状態

これにより、複数の BLE マウスを順番に接続しても前のマウスの解析結果を引きずらないようにしています。

## 制限事項

- BLE HID マウスを対象にしています。クラシック Bluetooth マウスには対応しません。
- キーボードや複合 HID デバイス全体の中継は目的にしていません。
- 特殊なベンダー独自機能は転送しません。
- 基本操作は左/右/中クリック、戻る/進む、X/Y 移動、ホイールを対象にしています。

## ライセンス

MIT License
