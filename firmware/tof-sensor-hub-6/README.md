# TOFセンサー(I2Cマルチプレクサ使用)

## 使用パーツ

* [Wio Terminal](https://www.switch-science.com/catalog/6360/)
* [Wio Terminal バッテリーベース](https://www.switch-science.com/catalog/6816/)
* [ToF レーザーモジュール(VL53L0X)](https://www.amazon.co.jp/gp/product/B08NDN4L9H/) 6個
* [Grove 8チャンネルI2Cマルチプレクサ/I2Cハブ(TCA9548A)](https://eleshop.jp/shop/g/gK2C313/)
* [USBシリアル変換モジュール(通販コード:M-11007)](https://akizukidenshi.com/catalog/g/gM-11007/)

## 接続図

![Breadboad](./images/breadboard.png)


## 測定値をパソコンで確認

Linuxの場合:

minicomをインストールして、以下のコマンドを実行します。

```bash
minicom -D /dev/ttyUSB0
```