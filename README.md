# Tarf

マイクロ・マウスを制作するプロジェクトです。

https://user-images.githubusercontent.com/10256609/150635355-fdfd84de-0ae6-4175-8a13-437b662bb9f7.mp4

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
| 9        | TimerContorllerの割込みを使って[3つのステッピングモータを別々の回転数で駆動](./firmware/stepper-motor-a4988-timer)させてみる。 |
| 10       | [試作1号機で往復運動](./firmware/reciprocating-motion)をさせてみる。                |
| 11       | [往復運動をタイマー割込み](./firmware/reciprocating-timer)でさせてみる。            |
| 12       | [三角形の軌跡](./firmware/triangle-motion)を描くように移動させる。                  |
| 13       | 段々と速度を上げる[加速運動](./firmware/accelaration)をさせる。                     |
| 14       | [TOF 距離センサ](./firmware/tof-sensor/)を接続する。                                |
| 15       | [LCDに文字](./firmware/lcd-display/)を表示する。                                    |
| 16       | [TOF 距離センサを6つ](./firmware/tof-sensor-hub-6/)接続する。                       |
| 17       | [TOF 距離センサで連続測定モード](./firmware/tof-sensor-interrupt/)で測定する。      |
| 18       | [2マスの直線だけの迷路](./firmware/straight-line-maze/)を走行する。                 |
| 19       | [L字型迷路](./firmware/l-shaped-maze/)をデッド・レコニングで走行する。              |
| 20       | [長い直線迷路](./firmware/long-straight-maze/)を補正しながら走行する。              |
| 21       | [スタート位置で向きと位置を調整](./firmware/adjust-position/)してから走行する。     |
| 22       | [4マスの迷路](./firmware/quad-cell-maze/)を走行する。                               |
| 23       | 9マスの[渦巻き状の迷路](./firmware/spiral-maze/)を走行する。                        |
| 23       | 9マスの[迷路を描画](./firmware/map-display/)してみる。                              |
| 23       | 9マスの渦巻き状の迷路を[現在地を表示](./firmware/current-pose/)しながら走行する。   |
| 24       | 9マスの渦巻き状の迷路を[側面の壁を表示](./firmware/wall-state/)しながら走行する。   |
| 25       | [BNO055 9軸センサ](./firmware/9axis-sensor/)を接続する。                            |
| 26       | [UARTにHello, worldを出力](./firmware/uart/)する。                                  |
| 25       | [9軸センサをI2Cハブ](./firmware/9axis-sensor-hub/)に接続する。                      |
| ***      | 以降、設計見直し                                                                    |
| 26       | [内蔵LEDでLチカ(その3)](./firmware/l-chika-3)をしてみる。                           |


Body

| ステップ | 内容                                                                                |
| -------- | ----------------------------------------------------------------------------------- |
| 1        | [ステッピングモータを固定](./cad/core1_3_motor)                                     |
| 2        | [ステッピングモータを固定の向きを変更](./cad/core1_3_motor2)                        |
| 3        | [ステッピングモータとWio Terminal部分との接続部を追加](./cad/core1_3_motor3)        |
| 4        | [Wio Terminalとバッテリーの収納部](./cad/core2)                                     |
| 5        | [オムニホイールのハブを作成](./cad/wheel_hub)                                       |
| 6        | [TOF距離センサを搭載](./cad/core1_motor_sensor_4)                                   |
| 7        | [Wio Terminalとバッテリーの収納部にI2Cハブを搭載](./cad/core2_3)                    |
| ***      | 以降大幅な設計変更                                                                  |
| 8        | [TOF距離センサの位置を変更](./cad/core1_motor_sensor_5/)                            |
| 9        | [Wio Terminalとバッテリーの格納部の下部の長さ調整](./cad/core2_5/)                  |


curcuit
| ステップ | 内容                                                                                |
| -------- | ----------------------------------------------------------------------------------- |
| 1 | [回路図](./circuit/universal_board_both.svg)  |

## 参考情報

* embedded-halクレートの[RustDoc](https://docs.rs/embedded-hal/0.2.6/embedded_hal/)
* ATSAMDマイコンのRustクレートの[リポジトリ](https://github.com/atsamd-rs/atsamd)  
  Wio Terminalのクレートもこの中
* atsamd51pのRustクレートの[RustDoc](https://docs.rs/atsamd51p/0.9.0/atsamd51p/)
* wio_terminalクレートの[RustDoc](https://docs.rs/wio_terminal/0.3.0/wio_terminal/)

* VL53L0Xの[使い方](./docs/VL53L0X.md)
