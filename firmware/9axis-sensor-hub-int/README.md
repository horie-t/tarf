# 9軸センサーをI2Cハブ経由で接続し、割込みで測定する

2023-02-07: 上手く割込みが動作しない

## 使用パーツ

* [Wio Terminal](https://www.switch-science.com/catalog/6360/)
* [Wio Terminal バッテリーベース](https://www.switch-science.com/catalog/6816/)
* [ＢＮＯ０５５使用　９軸センサーフュージョンモジュールキット](https://akizukidenshi.com/catalog/g/gK-16996/)
* [Grove 8チャンネルI2Cマルチプレクサ/I2Cハブ(TCA9548A)](https://eleshop.jp/shop/g/gK2C313/)
* [USBシリアル変換モジュール(通販コード:M-11007)](https://akizukidenshi.com/catalog/g/gM-11007/)

## 測定値をパソコンで確認

Linuxの場合:

minicomをインストールして、以下のコマンドを実行します。

```bash
minicom -D /dev/ttyUSB0
```