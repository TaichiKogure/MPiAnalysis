# remote_control_pkgVer2 インストールガイド

このガイドでは、MentorPiロボットをPCからリモート制御するための`remote_control_pkgVer2`パッケージのインストールと使用方法について説明します。

## 目次

1. [前提条件](#1-前提条件)
2. [ロボット側のセットアップ](#2-ロボット側のセットアップ)
3. [PC側のセットアップ](#3-pc側のセットアップ)
4. [使用方法](#4-使用方法)
5. [トラブルシューティング](#5-トラブルシューティング)

## 1. 前提条件

### ロボット側（Raspberry Pi）
- Raspberry Pi 5（または互換性のあるモデル）
- ROS2 Foxy以上がインストールされていること
- MentorPiCoreソフトウェアがインストールされていること
- WiFiアダプタ（内蔵または外付け）
- カメラ（Orbbec Dabai DCWなど）
- LiDARセンサー（LD19、A1、G4など）

### PC側
- Ubuntu 20.04以上またはWindows 10/11（WSL2を使用）
- ROS2 Foxy以上がインストールされていること
- 以下のPythonパッケージがインストールされていること：
  - opencv-python
  - numpy
  - matplotlib
  - rclpy

## 2. ロボット側のセットアップ

### 2.1 パッケージのインストール

1. ロボット側のターミナルを開きます。

2. ROS2ワークスペースのソースディレクトリに移動します：
   ```bash
   cd ~/ros2_ws/src
   ```

3. パッケージをクローンまたはコピーします：
   ```bash
   # GitHubからクローンする場合
   git clone https://github.com/your-username/remote_control_pkgVer2.git
   
   # または、USBドライブなどからコピーする場合
   cp -r /path/to/remote_control_pkgVer2 .
   ```

4. ワークスペースのルートディレクトリに戻り、パッケージをビルドします：
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select remote_control_pkgVer2
   ```

5. 環境をソースします：
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 2.2 自動起動の設定（オプション）

システム起動時に自動的にリモート制御ノードを起動するには、以下の手順を実行します：

1. systemdサービスファイルを作成します：
   ```bash
   sudo nano /etc/systemd/system/robot_control.service
   ```

2. 以下の内容を入力します：
   ```
   [Unit]
   Description=MentorPi Robot Control Node
   After=network.target

   [Service]
   User=ubuntu
   ExecStart=/bin/bash -c "source /opt/ros/foxy/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch remote_control_pkgVer2 robot_control.launch.py"
   Restart=on-failure
   RestartSec=5

   [Install]
   WantedBy=multi-user.target
   ```

3. サービスを有効にします：
   ```bash
   sudo systemctl enable robot_control.service
   sudo systemctl start robot_control.service
   ```

4. サービスのステータスを確認します：
   ```bash
   sudo systemctl status robot_control.service
   ```

## 3. PC側のセットアップ

### 3.1 パッケージのインストール

1. PC側のターミナルを開きます。

2. ROS2ワークスペースのソースディレクトリに移動します：
   ```bash
   cd ~/ros2_ws/src
   ```

3. パッケージをクローンまたはコピーします：
   ```bash
   # GitHubからクローンする場合
   git clone https://github.com/your-username/remote_control_pkgVer2.git
   
   # または、USBドライブなどからコピーする場合
   cp -r /path/to/remote_control_pkgVer2 .
   ```

4. ワークスペースのルートディレクトリに戻り、パッケージをビルドします：
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select remote_control_pkgVer2
   ```

5. 環境をソースします：
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 3.2 必要なPythonパッケージのインストール

以下のコマンドを実行して、必要なPythonパッケージをインストールします：

```bash
pip3 install opencv-python numpy matplotlib
```

## 4. 使用方法

### 4.1 ロボット側の起動

1. ロボットの電源を入れ、起動を待ちます。

2. 自動起動を設定していない場合は、以下のコマンドを実行してロボット制御ノードを起動します：
   ```bash
   ros2 launch remote_control_pkgVer2 robot_control.launch.py
   ```

3. 特定のパラメータを変更する場合は、以下のように引数を指定します：
   ```bash
   ros2 launch remote_control_pkgVer2 robot_control.launch.py enable_depth_camera:=true camera_width:=640 camera_height:=480
   ```

### 4.2 PC側の起動

1. ロボットとPCが同じネットワークに接続されていることを確認します。

2. ロボットのIPアドレスを確認します。ロボット側で以下のコマンドを実行してIPアドレスを取得できます：
   ```bash
   hostname -I
   ```

3. PC側で以下のコマンドを実行して、PC制御ノードを起動します：
   ```bash
   ros2 launch remote_control_pkgVer2 pc_control.launch.py robot_ip:=192.168.149.1
   ```
   （`192.168.149.1`をロボットの実際のIPアドレスに置き換えてください）

4. 特定のパラメータを変更する場合は、以下のように引数を指定します：
   ```bash
   ros2 launch remote_control_pkgVer2 pc_control.launch.py robot_ip:=192.168.149.1 enable_depth_view:=true linear_speed:=0.3
   ```

### 4.3 ロボットの操作方法

PC側の制御インターフェースが起動したら、以下の方法でロボットを操作できます：

- **矢印キー**：ロボットの移動
  - ↑：前進
  - ↓：後退
  - ←：左回転
  - →：右回転
- **スペースキー**：停止
- **Escキー**：プログラム終了

インターフェース上のスライダーを使用して、線形速度と角速度を調整することもできます。

### 4.4 録画機能

カメラ映像を録画するには、インターフェース下部の「Record」ボタンをクリックします。録画を停止するには、再度ボタンをクリックします。録画ファイルは、PC制御ノードを起動したディレクトリに保存されます。

## 5. トラブルシューティング

### 5.1 接続の問題

**症状**: PC側がロボットに接続できない

**解決策**:
1. ロボットとPCが同じネットワークに接続されていることを確認します。
2. ロボットのIPアドレスが正しいことを確認します。
3. ファイアウォールの設定を確認し、ROS2の通信ポートが開放されていることを確認します。
4. ROS2のDomain IDが両方のマシンで同じであることを確認します。

### 5.2 カメラの問題

**症状**: カメラ映像が表示されない

**解決策**:
1. カメラが正しく接続されていることを確認します。
2. カメラトピックが正しいことを確認します。以下のコマンドでトピックを確認できます：
   ```bash
   ros2 topic list | grep image
   ```
3. カメラドライバーが起動していることを確認します：
   ```bash
   ros2 launch peripherals usb_cam.launch.py
   # または
   export DEPTH_CAMERA_TYPE=ascamera
   ros2 launch peripherals depth_camera.launch.py
   ```

### 5.3 LiDARの問題

**症状**: LiDARデータが表示されない

**解決策**:
1. LiDARが正しく接続されていることを確認します。
2. LiDARトピックが正しいことを確認します。以下のコマンドでトピックを確認できます：
   ```bash
   ros2 topic list | grep scan
   ```
3. LiDARドライバーが起動していることを確認します：
   ```bash
   ros2 launch peripherals lidar.launch.py
   ```

### 5.4 パフォーマンスの問題

**症状**: 映像や操作にラグがある

**解決策**:
1. カメラの解像度を下げてみてください：
   ```bash
   ros2 launch remote_control_pkgVer2 robot_control.launch.py camera_width:=160 camera_height:=120
   ```
2. 圧縮画像フォーマットを使用していることを確認します：
   ```bash
   ros2 launch remote_control_pkgVer2 robot_control.launch.py use_compressed_image:=true
   ```
3. ネットワーク接続の品質を確認します。可能であれば、有線接続を使用してください。

## 追加情報

詳細な情報や最新のアップデートについては、以下のリソースを参照してください：

- [remote_control_pkgVer2 GitHub リポジトリ](https://github.com/your-username/remote_control_pkgVer2)
- [変更履歴](./changelog.md)
- [MentorPi ドキュメント](https://mentorpi.example.com/docs)