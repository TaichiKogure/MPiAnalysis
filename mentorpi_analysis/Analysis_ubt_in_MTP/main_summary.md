# MentorPi ubuntu_in_MTP 分析サマリー

## 概要

このドキュメントは、`G:\01_MentorPi\MPiAnalysis\mentorpi_analysis\ubuntu_in_MTP`内のデータを分析した結果をまとめたものです。MentorPiロボットのDockerコンテナ（ubuntu_in_MTP）内の各ディレクトリの内容、機能、および使用方法について詳細な分析を提供します。

## 分析ファイル一覧

以下のファイルが分析結果として作成されました：

1. **[ros2_ws_summary.md](ros2_ws_summary.md)** - ROS2ワークスペースの分析
2. **[RRCLite_demo_summary.md](RRCLite_demo_summary.md)** - RRCLiteデモコードの分析
3. **[shared_summary.md](shared_summary.md)** - 共有設定ファイルの分析
4. **[software_summary.md](software_summary.md)** - ソフトウェアツールの分析
5. **[third_party_ros2_summary.md](third_party_ros2_summary.md)** - サードパーティROS2パッケージの分析
6. **[rviz2_summary.md](rviz2_summary.md)** - RViz2設定の分析
7. **[MentorPiCore_Ext_and_Docker_Relationship.md](MentorPiCore_Ext_and_Docker_Relationship.md)** - MentorPiCore_ExtとDockerコンテナの関係

## テストプログラム

以下のスタンドアロンテストプログラムが作成されました：

1. **[lidar_test.py](lidar_test.py)** - LiDARセンサーをテストするためのPythonスクリプト
2. **[camera_test.py](camera_test.py)** - カメラ（RGB/深度）をテストするためのPythonスクリプト

## 主要コンポーネントの概要

### ROS2ワークスペース (ros2_ws)

MentorPiロボットのメインROS2ワークスペースです。以下のような重要なパッケージが含まれています：

- **peripherals**: カメラ、LiDARなどの周辺機器を制御するパッケージ
- **driver**: ハードウェアドライバーパッケージ
- **navigation**: ナビゲーション機能を提供するパッケージ
- **slam**: SLAMアルゴリズムを実装するパッケージ
- **yolov5_ros2**: YOLOv5オブジェクト検出を実装するパッケージ

詳細は[ros2_ws_summary.md](ros2_ws_summary.md)を参照してください。

### RRCLite_demo

Robot Robot Controller (RRC) Liteのデモコードが含まれています。モーター、サーボ、センサーなどのハードウェアコンポーネントを制御するためのROS2ノードを提供します。

詳細は[RRCLite_demo_summary.md](RRCLite_demo_summary.md)を参照してください。

### software

画像収集、ラベリング、オブジェクト検出、サーボ制御などの高レベル機能を提供するアプリケーションとツールが含まれています。

詳細は[software_summary.md](software_summary.md)を参照してください。

### third_party_ros2

MentorPiロボットが使用するサードパーティのROS2パッケージが含まれています。LiDARドライバー、カメラドライバー、ナビゲーションアルゴリズムなどが含まれています。

詳細は[third_party_ros2_summary.md](third_party_ros2_summary.md)を参照してください。

## システムアーキテクチャ

MentorPiロボットシステムは、ホストシステム（MentorPiCore_Ext）とDockerコンテナ（ubuntu_in_MTP）の2層アーキテクチャを採用しています：

- **MentorPiCore_Ext**: Raspberry Pi本体上で動作するベースシステム
- **ubuntu_in_MTP**: Dockerコンテナとして実行されるUbuntuベースのROS2環境

この2層アーキテクチャにより、ROS2環境の分離、移植性の向上、バージョン管理の容易さ、リソース管理の柔軟性などの利点があります。

詳細は[MentorPiCore_Ext_and_Docker_Relationship.md](MentorPiCore_Ext_and_Docker_Relationship.md)を参照してください。

## LiDARとカメラの使用方法

### LiDAR

MentorPiロボットは主にLD19 LiDARを使用しています。以下のコマンドでLiDARを起動できます：

```bash
# LiDARを起動
ros2 launch peripherals lidar.launch.py

# LiDARデータを可視化
ros2 launch peripherals lidar_view.launch.py
```

スタンドアロンテストプログラム[lidar_test.py](lidar_test.py)を使用して、LiDARの機能をテストすることもできます。

### カメラ

MentorPiロボットはUSBカメラと深度カメラ（ascamera）の両方をサポートしています。以下のコマンドでカメラを起動できます：

```bash
# USBカメラを起動
ros2 launch peripherals usb_cam.launch.py

# 深度カメラを起動
export DEPTH_CAMERA_TYPE=ascamera
ros2 launch peripherals depth_camera.launch.py
```

スタンドアロンテストプログラム[camera_test.py](camera_test.py)を使用して、カメラの機能をテストすることもできます。

## 結論

MentorPiロボットのubuntu_in_MTPコンテナは、ROS2を中心とした包括的なロボット制御環境を提供しています。様々なセンサー（LiDAR、カメラなど）とアクチュエータ（モーター、サーボなど）を制御するためのパッケージが含まれており、ナビゲーション、SLAM、コンピュータビジョンなどの高度な機能も実装されています。

このシステムは、Raspberry Pi上のホストシステム（MentorPiCore_Ext）とDockerコンテナ（ubuntu_in_MTP）の2層アーキテクチャを採用しており、これにより柔軟性と拡張性が向上しています。

この分析結果とテストプログラムを活用することで、MentorPiロボットシステムの理解、開発、およびカスタマイズが容易になります。