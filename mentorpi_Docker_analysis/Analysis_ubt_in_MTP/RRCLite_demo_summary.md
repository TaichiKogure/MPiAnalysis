# RRCLite_demo ディレクトリ分析

## 概要
RRCLite_demoディレクトリには、Robot Robot Controller (RRC) Liteのデモコードが含まれています。これらのPythonファイルは、ロボットのアクチュエータ（モーター、サーボなど）を制御し、センサー（IMU、バッテリー電圧など）からデータを読み取るためのROS2ノードを提供します。

## ファイル構造
```
RRCLite_demo/
├── battery_voltage_node.py       # バッテリー電圧を読み取るノード
├── bus_servos_read.py            # バスサーボの状態を読み取るユーティリティ
├── bus_servo_control_node.py     # バスサーボを制御するノード
├── bus_servo_control_speed_node.py # バスサーボの速度を制御するノード
├── bus_servo_state_reader_node.py # バスサーボの状態を読み取るノード
├── button_node.py                # ボタン入力を処理するノード
├── buzzer_control_node.py        # ブザーを制御するノード
├── imu_control_node.py           # IMUセンサーを制御するノード
├── led_control_node.py           # LEDを制御するノード
├── motor_control_node.py         # モーターを制御するノード
├── pwm_servo_controller_node.py  # PWMサーボを制御するノード
├── rgb_controller_node.py        # RGBライトを制御するノード
├── ros_robot_controller_sdk.py   # ロボットコントローラーSDK
└── servo_state_reader_node.py    # サーボの状態を読み取るノード
```

## 主要コンポーネント

### モーター制御
`motor_control_node.py`はロボットの移動用モーターを制御するためのROS2ノードを提供します。このノードは速度コマンドを受け取り、それをモーターの制御信号に変換します。

```python
# モーター制御ノードの基本的な使用方法
ros2 run <package_name> motor_control_node.py
```

### サーボ制御
複数のサーボ制御ノードが提供されています：
- `bus_servo_control_node.py`: シリアルバス経由でサーボを制御
- `pwm_servo_controller_node.py`: PWM信号でサーボを制御

```python
# バスサーボ制御ノードの基本的な使用方法
ros2 run <package_name> bus_servo_control_node.py
```

### センサーインターフェース
- `imu_control_node.py`: IMUセンサーからデータを読み取り、処理します
- `battery_voltage_node.py`: バッテリー電圧を監視します

```python
# IMU制御ノードの基本的な使用方法
ros2 run <package_name> imu_control_node.py
```

### ユーザーインターフェース要素
- `led_control_node.py`: LEDの制御
- `buzzer_control_node.py`: ブザーの制御
- `button_node.py`: ボタン入力の処理
- `rgb_controller_node.py`: RGBライトの制御

```python
# LED制御ノードの基本的な使用方法
ros2 run <package_name> led_control_node.py
```

## ROS Robot Controller SDK
`ros_robot_controller_sdk.py`は、ロボットのハードウェアコンポーネントとのインターフェースを提供する包括的なSDKです。このSDKは、上記のすべてのノードが使用する基本的な機能を提供します。

## 使用方法
これらのノードは通常、ROS2の起動ファイルを通じて起動されます。個々のノードは、特定のハードウェアコンポーネントをテストするために単独で実行することもできます。

```bash
# 例：モーター制御ノードを実行
ros2 run <package_name> motor_control_node.py

# 例：サーボ状態を読み取る
ros2 run <package_name> servo_state_reader_node.py
```

## 注意点
- これらのノードを使用するには、適切なハードウェア（モーター、サーボ、センサーなど）がロボットに接続されている必要があります
- 一部のノードは特定のハードウェア設定に依存している可能性があります
- 実際の使用では、これらのノードは通常、より高レベルの制御システムの一部として使用されます