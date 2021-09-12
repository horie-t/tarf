# Tarf

マイクロ・マウスを制作するプロジェクトです。

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

| ステップ | 内容                                                                                |
| -------- | ----------------------------------------------------------------------------------- |
| 1        | [内蔵LEDでLチカ](./firmware/l-chika)をしてみる。                                    |
| 2        | ブレッドボードに接続して、[外部のLEDでLチカ](./firmware/l-chika-external)してみる。 | 
| 3        | L6470モータドライバを使って、[ステッピングモータを駆動](./firmware/stepper-motor)させてみる。|
| 4        | L6470モータドライバを使って、[ステッピングモータをステップ数指定で駆動](./firmware/stepper-motor-count)させてみる。|
| 5        | 外部の[スイッチからの割込み](./firmware/interrupt-external)を受け付ける。           |
| 6        | [ステッピングモータの動作完了を割込みで処理する](./firmware/stepper-motor-interrupt)。 |

## 参考情報

* embedded-halクレートの[RustDoc](https://docs.rs/embedded-hal/0.2.6/embedded_hal/)
* ATSAMDマイコンのRustクレートの[リポジトリ](https://github.com/atsamd-rs/atsamd)  
  Wio Terminalのクレートもこの中
* atsamd51pのRustクレートの[RustDoc](https://docs.rs/atsamd51p/0.9.0/atsamd51p/)
* wio_terminalクレートの[RustDoc](https://docs.rs/wio_terminal/0.3.0/wio_terminal/)
