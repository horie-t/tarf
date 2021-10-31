# Tarf

マイクロ・マウスを制作するプロジェクトです。

https://user-images.githubusercontent.com/10256609/137706155-5c6c29f7-b0da-4c39-ac3d-75bc74e54c84.mp4

Tarfは以下の特徴を持ちます。

* 筐体は3Dプリンタで制作
* 制御には、Wio Terminalを使用
* 制御用のFrimwareは、Rustで実装

## Rust開発環境のセットアップ

Ubuntu上で開発しています。

開発に必須のパッケージをインストールします。

```bash
sudo apt install build-essential git
```

Rustのツール・チェーンをインストールします。

[rustup](https://rustup.rs/) の指示に従ってインストールします。


クロスビルド用のツール・チェーンをインストールします。

```bash
rustup target add thumbv7em-none-eabihf
```

Wio Terminal用に必要なパッケージをインストールします。

```bash
sudo apt install libusb-1.0-0-dev libsdl2-dev libssl-dev
```

シリアルデバイスの読み書き権限の設定

```bash
sudo adduser $USER dialout
```

上記で、Wio Terminalにプログラムの書き込みができない場合は以下のファイルを作成します。

/etc/udev/rules.d/99-seeed-boards.rules
```
ATTRS{idVendor}=="2886", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2886", MODE="0666"
```

作成後、以下のコマンドを実行します。

```bash
sudo udevadm control --reload-rules
```

## 開発記録

Firmware

| ステップ | 内容                                                                                |
| -------- | ----------------------------------------------------------------------------------- |
| 1        | [内蔵LEDでLチカ](./firmware/l-chika)をしてみる。                                    |
| 2        | ブレッドボードに接続して、[外部のLEDでLチカ](./firmware/l-chika-external)してみる。 | 
| 3        | L6470モータドライバを使って、[ステッピングモータを駆動](./firmware/stepper-motor)させてみる。|
| 4        | L6470モータドライバを使って、[ステッピングモータをステップ数指定で駆動](./firmware/stepper-motor-count)させてみる。|
| 5        | 外部の[スイッチからの割込み](./firmware/interrupt-external)を受け付ける。           |
| 6        | [ステッピングモータの動作完了を割込みで処理する](./firmware/stepper-motor-interrupt)。 |
| 7        | A4988モータドライバを使って、[ステッピングモータを駆動](./firmware/stepper-motor-a4988)させてみる。 |
| 8        | [3つのステッピングモータを駆動](./firmware/stepper-motor-a4988-3motor)させてみる。 |
| 9        | TimerContorllerの割込みを使って[3つのステッピングモータを別々の回転数で駆動](./firmware/stepper-motor-a4988-interrupt)させてみる。 |
| 10       | [試作1号機で往復運動](./firmware/reciprocating-motion)をさせてみる。 |
| 11       | [往復運動をタイマー割込み](./firmware/reciprocating-timer)でさせてみる。 |
| 12       | [三角形の軌跡](./firmware/reciprocating-timer)を描くように移動させる。 |

Body

| ステップ | 内容                                                                                |
| -------- | ----------------------------------------------------------------------------------- |
| 1        | [ステッピングモータを固定](./cad/core1_3_motor)                                     |
| 2        | [ステッピングモータを固定の向きを変更](./cad/core1_3_motor2)                        |
| 3        | [ステッピングモータとWio Terminal部分との接続部を追加](./cad/core1_3_motor3)        |
| 4        | [Wio Terminalとバッテリーの収納部](./cad/core2)                                     |


## 参考情報

* embedded-halクレートの[RustDoc](https://docs.rs/embedded-hal/0.2.6/embedded_hal/)
* ATSAMDマイコンのRustクレートの[リポジトリ](https://github.com/atsamd-rs/atsamd)  
  Wio Terminalのクレートもこの中
* atsamd51pのRustクレートの[RustDoc](https://docs.rs/atsamd51p/0.9.0/atsamd51p/)
* wio_terminalクレートの[RustDoc](https://docs.rs/wio_terminal/0.3.0/wio_terminal/)
