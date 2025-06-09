# RViz2 (.rviz2) ディレクトリ分析

## 概要
.rviz2ディレクトリには、ROS2の可視化ツールであるRViz2の設定ファイルが含まれています。RViz2は、ロボットのセンサーデータ、ロボットの状態、および環境マップなどを視覚的に表示するためのツールです。これらの設定ファイルは、MentorPiロボットのセンサーデータを効果的に可視化するためにカスタマイズされています。

## ディレクトリ構造
```
.rviz2/
├── default.rviz             # デフォルトのRViz2設定ファイル
└── persistent_settings      # 永続的な設定ファイル
```

## 主要ファイル

### default.rviz
このファイルは、RViz2のデフォルト設定を定義します。以下のような要素が含まれています：
- 表示パネルの配置
- 表示するトピックとその視覚化方法
- カメラの視点と設定
- グリッドやその他の視覚的要素の設定

この設定ファイルは、MentorPiロボットの一般的な使用シナリオに合わせて最適化されています。

### persistent_settings
このファイルには、RViz2の永続的な設定が保存されています。これには、ウィンドウのサイズと位置、最後に使用した設定ファイル、およびその他のユーザー固有の設定が含まれます。

## RViz2の使用方法

### 基本的な起動
```bash
# デフォルト設定でRViz2を起動
ros2 run rviz2 rviz2

# 特定の設定ファイルでRViz2を起動
ros2 run rviz2 rviz2 -d /path/to/.rviz2/default.rviz
```

### 一般的な可視化シナリオ

#### LiDARデータの可視化
```bash
# LiDARを起動
ros2 launch peripherals lidar.launch.py

# RViz2でLiDARデータを可視化
ros2 run rviz2 rviz2
```

RViz2内で、「Add」ボタンをクリックし、「LaserScan」を選択して、トピックとして「/scan」を指定します。

#### カメラ映像の可視化
```bash
# カメラを起動
ros2 launch peripherals usb_cam.launch.py

# RViz2でカメラ映像を可視化
ros2 run rviz2 rviz2
```

RViz2内で、「Add」ボタンをクリックし、「Image」を選択して、トピックとして「/ascamera/camera_publisher/rgb0/image」を指定します。

#### ロボットモデルの可視化
```bash
# ロボットの状態発行ノードを起動
ros2 launch bringup robot_state_publisher.launch.py

# RViz2でロボットモデルを可視化
ros2 run rviz2 rviz2
```

RViz2内で、「Add」ボタンをクリックし、「RobotModel」を選択します。

#### ナビゲーションマップの可視化
```bash
# ナビゲーションを起動
ros2 launch navigation navigation.launch.py

# RViz2でナビゲーションマップを可視化
ros2 run rviz2 rviz2
```

RViz2内で、「Add」ボタンをクリックし、「Map」を選択して、トピックとして「/map」を指定します。

## カスタム設定の作成と保存

RViz2の設定をカスタマイズするには：
1. RViz2を起動します
2. 必要なディスプレイパネルを追加し、設定します
3. 「File」→「Save Config As」を選択します
4. 設定ファイルを保存する場所を指定します

```bash
# カスタム設定でRViz2を起動
ros2 run rviz2 rviz2 -d /path/to/your/custom_config.rviz
```

## 注意点
- RViz2を使用するには、表示するデータを提供するROS2ノードが実行されている必要があります
- 3Dモデルの可視化には、適切なURDFファイルが必要です
- 高解像度のカメラ映像やポイントクラウドを表示する場合、システムのグラフィック性能に依存します
- RViz2の設定は、特定のトピック名やフレームIDに依存しているため、これらが変更された場合は設定の更新が必要になる場合があります