# MentorPiロボット遠隔制御システム マニュアル

## 1. 概要

このマニュアルでは、ROS2を搭載したMentorPiロボットを外部PCから遠隔制御するためのシステムの設定方法と使用方法について説明します。このシステムを使用することで、以下のことが可能になります：

- WiFi経由でロボットを遠隔操作する
- カメラ映像をリアルタイムで表示する
- LiDARスキャンデータを可視化する
- キーボードでロボットを制御する

## 2. システム要件

### 2.1 ロボット側（Raspberry Pi）

- Raspberry Pi 5（または互換性のあるモデル）
- ROS2 Foxy以上
- MentorPiCoreソフトウェア
- WiFiアダプタ（内蔵または外付け）
- カメラ（Orbbec Dabai DCWなど）
- LiDARセンサー（LD19、A1、G4など）

### 2.2 PC側

- Ubuntu 20.04以上またはWindows 10/11（WSL2を使用）
- ROS2 Foxy以上
- Python 3.8以上
- 必要なPythonパッケージ：
  - opencv-python
  - numpy
  - matplotlib
  - rclpy

## 3. インストール手順

### 3.1 ロボット側のセットアップ

1. **remote_control_pkgパッケージのインストール**

   ```bash
   # ロボット側のターミナルで実行
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/remote_control_pkg.git
   cd ~/ros2_ws
   colcon build --packages-select remote_control_pkg
   source ~/ros2_ws/install/setup.bash
   ```

2. **自動起動の設定（オプション）**

   システム起動時に自動的にリモート制御ノードを起動するには、以下の手順を実行します：

   ```bash
   # systemdサービスファイルの作成
   sudo nano /etc/systemd/system/robot_control.service
   ```

   以下の内容を入力します：

   ```
   [Unit]
   Description=Robot Control Node
   After=network.target

   [Service]
   User=ubuntu
   WorkingDirectory=/home/ubuntu
   ExecStart=/bin/bash -c "source /opt/ros/foxy/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch remote_control_pkg robot_control.launch.py"
   Restart=always
   RestartSec=5

   [Install]
   WantedBy=multi-user.target
   ```

   サービスを有効化します：

   ```bash
   sudo systemctl enable robot_control.service
   sudo systemctl start robot_control.service
   ```

### 3.2 PC側のセットアップ

1. **remote_control_pkgパッケージのインストール**

   ```bash
   # PC側のターミナルで実行
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/remote_control_pkg.git
   cd ~/ros2_ws
   colcon build --packages-select remote_control_pkg
   source ~/ros2_ws/install/setup.bash
   ```

2. **必要なPythonパッケージのインストール**

   ```bash
   pip install opencv-python numpy matplotlib
   ```

## 4. ネットワーク設定

### 4.1 ロボットのWiFi設定

MentorPiロボットは、以下の2つのWiFiモードをサポートしています：

1. **APモード（アクセスポイントモード）**
   - ロボットが独自のWiFiネットワークを作成し、PCがそれに接続します
   - デフォルトのSSID: `HW-XXXXXXXX`（XXXXXXXXはCPUシリアル番号）
   - デフォルトのパスワード: `hiwonder`
   - デフォルトのIPアドレス: `192.168.149.1`

2. **STAモード（ステーションモード）**
   - ロボットが既存のWiFiネットワークに接続します
   - IPアドレスはDHCPによって割り当てられます

#### 4.1.1 APモードの設定

1. ロボットのボタン1を長押しして、WiFi設定をリセットします
2. ロボットが自動的にAPモードで起動します（LED1が点灯し、LED2が点滅します）
3. PCからロボットのWiFiネットワーク（`HW-XXXXXXXX`）に接続します

#### 4.1.2 STAモードの設定

1. `/etc/wifi/wifi_conf.py`ファイルを編集します：

   ```bash
   sudo nano /etc/wifi/wifi_conf.py
   ```

   以下の内容を設定します：

   ```python
   WIFI_MODE = 2
   WIFI_STA_SSID = "your_wifi_ssid"
   WIFI_STA_PASSWORD = "your_wifi_password"
   ```

2. WiFiサービスを再起動します：

   ```bash
   sudo systemctl restart wifi.service
   ```

3. ロボットが指定したWiFiネットワークに接続します（LED1とLED2が両方点灯します）

### 4.2 PC側のネットワーク設定

1. **APモードの場合**：
   - PCをロボットのWiFiネットワーク（`HW-XXXXXXXX`）に接続します
   - ロボットのIPアドレスは `192.168.149.1` です

2. **STAモードの場合**：
   - PCとロボットを同じWiFiネットワークに接続します
   - ロボットのIPアドレスを確認するには、以下のコマンドをロボット側で実行します：
     ```bash
     ip addr show wlan0 | grep "inet "
     ```

## 5. 使用方法

### 5.1 ロボット側の起動

1. ロボット制御ノードを起動します：

   ```bash
   # ロボット側のターミナルで実行
   ros2 launch remote_control_pkg robot_control.launch.py
   ```

   または、自動起動を設定している場合は、ロボットを再起動するだけで制御ノードが自動的に起動します。

### 5.2 PC側の起動

1. PC制御ノードを起動します：

   ```bash
   # PC側のターミナルで実行
   ros2 launch remote_control_pkg pc_control.launch.py robot_ip:=192.168.149.1
   ```

   注意：`robot_ip`パラメータには、ロボットの実際のIPアドレスを指定してください。

2. GUIウィンドウが表示され、カメラ映像とLiDARスキャンデータが表示されます。

### 5.3 ロボットの制御

ロボットは、以下のキーボードコマンドで制御できます：

- **↑（上矢印）**：前進
- **↓（下矢印）**：後退
- **←（左矢印）**：左回転
- **→（右矢印）**：右回転
- **スペース**：停止
- **Esc**：終了

## 6. トラブルシューティング

### 6.1 接続の問題

1. **ロボットに接続できない場合**：
   - ロボットとPCが同じネットワークに接続されていることを確認してください
   - ロボットのIPアドレスが正しいことを確認してください
   - ファイアウォールがROS2通信をブロックしていないことを確認してください

2. **カメラ映像が表示されない場合**：
   - カメラが正しく接続されていることを確認してください
   - カメラトピックが正しいことを確認してください：
     ```bash
     # ロボット側で実行
     ros2 topic list | grep image
     ```

3. **LiDARデータが表示されない場合**：
   - LiDARが正しく接続されていることを確認してください
   - LiDARトピックが正しいことを確認してください：
     ```bash
     # ロボット側で実行
     ros2 topic list | grep scan
     ```

### 6.2 パフォーマンスの問題

1. **映像の遅延が大きい場合**：
   - カメラの解像度を下げてみてください（`robot_control.launch.py`の`camera_resolution`パラメータを変更）
   - WiFi接続の品質を確認してください
   - ロボットとPCの距離を近づけてみてください

2. **制御の応答が遅い場合**：
   - WiFi接続の品質を確認してください
   - 他のネットワークトラフィックを減らしてみてください
   - QoS設定を調整してみてください（`pc_control_node.py`の`qos_profile`を変更）

## 7. 高度な設定

### 7.1 カスタムトピックの使用

デフォルトでは、システムは以下のトピックを使用します：

- カメラ：`/depth_cam/rgb/image_raw`
- LiDAR：`/scan`
- 速度コマンド：`/controller/cmd_vel`

これらのトピックを変更するには、起動ファイルのパラメータを変更します：

```bash
# ロボット側
ros2 launch remote_control_pkg robot_control.launch.py camera_topic:=/custom/camera lidar_topic:=/custom/lidar cmd_vel_topic:=/custom/cmd_vel

# PC側
ros2 launch remote_control_pkg pc_control.launch.py camera_topic:=/robot_control/camera lidar_topic:=/robot_control/lidar cmd_vel_topic:=/robot_control/cmd_vel
```

### 7.2 複数のロボットの制御

複数のロボットを制御するには、各ロボットに異なるネームスペースを設定します：

```bash
# ロボット1
ros2 launch remote_control_pkg robot_control.launch.py __ns:=/robot1

# ロボット2
ros2 launch remote_control_pkg robot_control.launch.py __ns:=/robot2

# PC側（ロボット1を制御）
ros2 launch remote_control_pkg pc_control.launch.py robot_ip:=192.168.149.1 camera_topic:=/robot1/robot_control/camera lidar_topic:=/robot1/robot_control/lidar cmd_vel_topic:=/robot1/robot_control/cmd_vel
```

## 8. 参考情報

- [ROS2公式ドキュメント](https://docs.ros.org/en/foxy/index.html)
- [MentorPiCoreドキュメント](https://github.com/your-organization/MentorPiCore)
- [OpenCVドキュメント](https://docs.opencv.org/)
- [Matplotlibドキュメント](https://matplotlib.org/stable/contents.html)