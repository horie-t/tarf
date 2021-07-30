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

## 参考情報

* ATSAMDマイコンのRustクレートの[リポジトリ](https://github.com/atsamd-rs/atsamd)  
  Wio Terminalのクレートもこの中

* wio_terminalクレートの[RustDoc](https://docs.rs/wio_terminal/0.3.0/wio_terminal/)