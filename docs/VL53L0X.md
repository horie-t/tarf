# VL53L0Xの簡単な説明

## はじめに

VL53L0Xはレーザー光を利用したTOF型距離センサです。

最大測定距離は以下の通り。

| Target reflectance level(full FOV) | Conditions | Indoor(2)  | Outdoor overcast (2) |
| ---------------------------------- | ---------- | ---------- | -------------------- |
| White target (88%)                 | Typical    | 200cm+ (1) | 80cm                 |
|                                    | Minimum    | 120cm      | 60cm                 |
| Grey target (17%)                  | Typical    | 80cm       | 50cm                 |
|                                    | Minimum    | 70cm       | 40cm                 |

(VL53L0X Datasheet P.26より)

ホストとはI2Cで接続されます。I2Cバスの最大速度は400kbpsです。レジスタの仕様は公開されていない。

## 距離の測定

測定方法は以下の2つの方法です。

* ポーリングモード
* 割込みモード: 値が利用可能になると割込みピン(GPIO1)がホストに割込みを送信します。

## 測定シーケンス

ポーリングモードで測定する場合はAPI呼び出し後、timing buget(デフォルトは33msec) の時間以内で測定します。

## 測定プロファイル

| Range profile | Range timing budget | Typical performance       | Typical application |
| ------------- | ------------------- | ------------------------- | ------------------- |
| Default mode  | 30 ms               | 1.2 m, accuracy omit      | Standard            |
| High accuracy | 200 ms              | 1.2 m, accuracy < +/- 3 % | Precise measurement |
| Long range    | 33 ms               | 2 m, accuracy omit        | Long ranging, only for dark conditions  (no IR) |
| High speed    | 20 ms               | 1.2 m, accuracy +/- 5 %   | High speed where accuracy is not priority |

(VL53L0X Datasheet P.28より)

timing budgetは、最小 20 ms, 最大 1000 ms。

## vl53l0xクレートの使用方法

Cargo.tomlの設定は [creates.io](https://crates.io/crates/vl53l0x) を参照。

**【インスタンスの生成】**
VL53L0x::new(i2c)

**【単発での測定】**

1. read_range_single_millimeters_blocking()


**【割込みでの連続測定】**

1. start_continuous(timing budgetのミリ秒)
2. 割込み発生
3. read_range_continuous_millimeters_blocking() もしくは read_range_mm()
4. 2, 3の繰り返し
5. (測定停止の場合)stop_continuous()

**【timing budget】**

get_measurement_timing_budget(ミリ秒)
set_measurement_timing_budget(ミリ秒)





使用例

* ポーリングモードの[例](../firmware/tof-sensor/src/main.rs)
