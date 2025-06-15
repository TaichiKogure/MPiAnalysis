#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MentorPi LiDAR テストプログラム

このスクリプトは、MentorPiロボットのLiDARセンサー（LD19）からデータを読み取り、
簡単な処理と可視化を行うためのスタンドアロンテストプログラムです。

使用方法:
1. MentorPiロボットのDockerコンテナ内で実行します:
   $ cd /path/to/save/this/script
   $ python3 lidar_test.py

2. 別のターミナルでLiDARを起動しておく必要があります:
   $ ros2 launch peripherals lidar.launch.py

注意:
- このスクリプトを実行するには、ROS2環境が必要です
- matplotlib, numpy, rclpy パッケージがインストールされている必要があります
- LiDARデバイス（/dev/ldlidar）が接続されている必要があります
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
import time

class LidarTestNode(Node):
    """
    LiDARデータを受信して処理するROS2ノード
    """
    
    def __init__(self):
        """
        ノードの初期化とLaserScanトピックのサブスクライバーを設定
        """
        super().__init__('lidar_test_node')
        
        # LiDARスキャンデータのサブスクライバーを作成
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # デフォルトのスキャントピック
            self.scan_callback,
            10)  # QoSプロファイル
        
        self.get_logger().info('LiDAR テストノードが起動しました。スキャンデータを待機中...')
        
        # 可視化用の設定
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_title('LiDAR スキャンデータ')
        self.ax.grid(True)
        
        # 最新のスキャンデータを保存
        self.latest_scan = None
        
        # 障害物検出のしきい値（メートル）
        self.obstacle_threshold = 0.5
        
        # 統計情報の初期化
        self.scan_count = 0
        self.start_time = time.time()

    def scan_callback(self, msg):
        """
        LaserScanメッセージを受信したときに呼び出されるコールバック関数
        
        Args:
            msg (LaserScan): 受信したLaserScanメッセージ
        """
        self.scan_count += 1
        self.latest_scan = msg
        
        # スキャンデータの基本情報を表示
        self.get_logger().info(f'スキャン #{self.scan_count} 受信: {len(msg.ranges)} ポイント')
        
        # 有効な範囲データを抽出（無限大や無効な値を除外）
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and r > 0.0]
        
        if valid_ranges:
            # 基本的な統計情報を計算
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            avg_range = sum(valid_ranges) / len(valid_ranges)
            
            self.get_logger().info(f'  距離範囲: 最小={min_range:.2f}m, 最大={max_range:.2f}m, 平均={avg_range:.2f}m')
            
            # 障害物検出（しきい値以下の距離にある点を数える）
            obstacles = [r for r in valid_ranges if r < self.obstacle_threshold]
            if obstacles:
                self.get_logger().warn(f'  警告: {len(obstacles)} 点の障害物が {self.obstacle_threshold}m 以内に検出されました')
        
        # 可視化を更新
        self.update_visualization()
        
        # 処理速度を計算（1秒ごと）
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 1.0:
            hz = self.scan_count / elapsed_time
            self.get_logger().info(f'処理速度: {hz:.1f} Hz')
            self.scan_count = 0
            self.start_time = time.time()

    def update_visualization(self):
        """
        matplotlib を使用してLiDARデータを可視化
        """
        if self.latest_scan is None:
            return
        
        # 極座標プロットのためのデータを準備
        scan = self.latest_scan
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        
        # 前回のプロットをクリア
        self.ax.clear()
        
        # 極座標プロットを作成
        self.ax.scatter(angles, scan.ranges, s=2, c='blue')
        
        # 障害物を強調表示（しきい値以下の点を赤で表示）
        obstacle_indices = [i for i, r in enumerate(scan.ranges) if not math.isinf(r) and r > 0.0 and r < self.obstacle_threshold]
        if obstacle_indices:
            obstacle_angles = [angles[i] for i in obstacle_indices]
            obstacle_ranges = [scan.ranges[i] for i in obstacle_indices]
            self.ax.scatter(obstacle_angles, obstacle_ranges, s=10, c='red')
        
        # プロットの設定
        self.ax.set_title('LiDAR スキャンデータ')
        self.ax.set_rmax(10.0)  # 最大表示範囲を10mに設定
        self.ax.grid(True)
        
        # プロットを更新
        plt.pause(0.001)

def main(args=None):
    """
    メイン関数
    """
    # ROS2の初期化
    rclpy.init(args=args)
    
    # LiDARテストノードの作成
    lidar_test_node = LidarTestNode()
    
    try:
        # ノードの実行
        rclpy.spin(lidar_test_node)
    except KeyboardInterrupt:
        # Ctrl+Cで終了
        lidar_test_node.get_logger().info('ユーザーによって終了されました')
    except Exception as e:
        # その他の例外
        lidar_test_node.get_logger().error(f'エラーが発生しました: {str(e)}')
    finally:
        # ノードの破棄
        lidar_test_node.destroy_node()
        # ROS2のシャットダウン
        rclpy.shutdown()
        # matplotlibのウィンドウを閉じる
        plt.close('all')

if __name__ == '__main__':
    main()