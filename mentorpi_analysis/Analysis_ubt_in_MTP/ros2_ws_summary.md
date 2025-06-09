# ROS2 Workspace (ros2_ws) 分析

## 概要
ros2_wsディレクトリはMentorPiロボットのメインROS2ワークスペースです。このワークスペースには、ロボットの機能を実装するための様々なROS2パッケージが含まれています。標準的なROS2ワークスペース構造（build、install、log、src）に従っています。

## ディレクトリ構造
```
ros2_ws/
├── build/       # ビルド成果物
├── install/     # インストールされたパッケージ
├── log/         # ログファイル
└── src/         # ソースコード
    ├── app/             # アプリケーションパッケージ
    ├── bringup/         # システム起動パッケージ
    ├── calibration/     # センサーキャリブレーションパッケージ
    ├── driver/          # ハードウェアドライバーパッケージ
    ├── example/         # サンプルコードパッケージ
    ├── interfaces/      # メッセージとサービスの定義
    ├── multi/           # マルチロボット関連パッケージ
    ├── navigation/      # ナビゲーションパッケージ
    ├── peripherals/     # 周辺機器パッケージ
    ├── simulations/     # シミュレーションパッケージ
    ├── slam/            # SLAMパッケージ
    └── yolov5_ros2/     # YOLOv5オブジェクト検出パッケージ
```

## 主要パッケージ

### peripherals
周辺機器（カメラ、LiDARなど）とのインターフェースを提供します。

#### 主要ファイル:
- **launch/lidar.launch.py**: LiDARセンサー（LD19）を起動するためのファイル
- **launch/usb_cam.launch.py**: USBカメラを起動するためのファイル
- **launch/depth_camera.launch.py**: 深度カメラを起動するためのファイル
- **peripherals/teleop_key_control.py**: キーボードによるテレオペレーション

#### LiDAR設定:
LiDARはLD19モデルを使用しており、以下の設定で動作します:
- デバイス: `/dev/ldlidar`
- ボーレート: 230400
- トピック名: `scan`
- フレームID: `base_laser`（デフォルト）

#### カメラ設定:
システムは2種類のカメラをサポートしています:

1. **USBカメラ**:
   - デバイス: `/dev/video0`
   - 解像度: 640x480
   - フレームレート: 30fps
   - ピクセルフォーマット: yuyv

2. **深度カメラ（ascamera）**:
   - 解像度: 640x480（深度とRGB）
   - フレームレート: 15fps
   - 設定ファイルパス: `/home/ubuntu/third_party_ros2/third_party_ws/src/ascamera/configurationfiles`

### driver
ロボットのハードウェアを制御するための低レベルドライバーを提供します。

### navigation
ロボットのナビゲーション機能を提供します。

### slam
Simultaneous Localization and Mapping（SLAM）機能を提供します。

### yolov5_ros2
YOLOv5を使用したオブジェクト検出機能を提供します。

## 使用方法

### LiDARの起動
```bash
# LD19 LiDARを起動
ros2 launch peripherals lidar.launch.py

# LiDARデータを可視化
ros2 launch peripherals lidar_view.launch.py
```

### カメラの起動
```bash
# USBカメラを起動
ros2 launch peripherals usb_cam.launch.py

# 深度カメラを起動（環境変数の設定が必要）
export DEPTH_CAMERA_TYPE=ascamera
ros2 launch peripherals depth_camera.launch.py
```

### テレオペレーション
```bash
# キーボードによるテレオペレーション
ros2 launch peripherals teleop_key_control.launch.py
```

## 注意点
- 環境変数`need_compile`と`DEPTH_CAMERA_TYPE`が一部の起動ファイルで使用されています
- カメラとLiDARのデバイスパスが正しいことを確認してください
- 一部のパッケージは外部依存関係（third_party_ros2ディレクトリ内）に依存しています