# 9軸センサー


## 使用パーツ

* [Wio Terminal](https://www.switch-science.com/catalog/6360/)
* [Wio Terminal バッテリーベース](https://www.switch-science.com/catalog/6816/)
* [ＢＮＯ０５５使用　９軸センサーフュージョンモジュールキット](https://akizukidenshi.com/catalog/g/gK-16996/)
* [USBシリアル変換モジュール(通販コード:M-11007)](https://akizukidenshi.com/catalog/g/gM-11007/)

## ブレッドボードで接続する場合の接続図

![Breadboad](./images/breadboard.png)


## 測定値をパソコンで確認

Linuxの場合:

minicomをインストールして、以下のコマンドを実行します。

```bash
minicom -D /dev/ttyUSB0
```