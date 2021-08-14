# Wio Terminalとブレッドボードを接続して、ステッピングモータを動かします。

## ブレッドボードとの接続

![Breadboad](./images/stepper_motor_breadboard.png)


Wio Teraminlのピンは、説明書参照。

| ピンNo. | 説明     | ASTAMD51P19A パッド |
| ------- | -------- | ------------------  |
| 19      | SPI_MOSI | PB02/SERCOM5.0      |
| 21      | SPI_MISO | PB00/SERCOM5.2      |
| 23      | SPI_SCLK | PB03/SERCOM5.1      |
| 24      | SPI_CS   | PB01/SERCOM5.3      |

## 参考情報

* [L6470使用　ステッピングモータードライブキットの取扱説明書](https://akizukidenshi.com/download/ds/akizuki/AE-L6470_20190118.pdf)
* [L6470データシート](https://akizukidenshi.com/download/ds/st/L6470.pdf)
* [Aruduinoでのステッピングモータの接続](https://github.com/laurb9/StepperDriver)
