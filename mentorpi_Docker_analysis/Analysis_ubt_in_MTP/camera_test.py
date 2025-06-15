#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MentorPi カメラ テストプログラム

このスクリプトは、MentorPiロボットのカメラから画像を読み取り、
簡単な処理と表示を行うためのスタンドアロンテストプログラムです。
USBカメラと深度カメラ（ascamera）の両方をサポートしています。

使用方法:
1. MentorPiロボットのDockerコンテナ内で実行します:
   $ cd /path/to/save/this/script
   $ python3 camera_test.py [--depth] [--topic TOPIC_NAME]

   オプション:
   --depth: 深度カメラモードを有効にします（デフォルトはRGBカメラモード）
   --topic TOPIC_NAME: 購読する画像トピックを指定します
                      （デフォルトはRGBカメラの場合は '/ascamera/camera_publisher/rgb0/image'、
                       深度カメラの場合は '/ascamera/camera_publisher/depth/image'）

2. 別のターミナルでカメラを起動しておく必要があります:
   # USBカメラの場合:
   $ ros2 launch peripherals usb_cam.launch.py
   
   # 深度カメラの場合:
   $ export DEPTH_CAMERA_TYPE=ascamera
   $ ros2 launch peripherals depth_camera.launch.py

注意:
- このスクリプトを実行するには、ROS2環境が必要です
- OpenCV (cv2), numpy, rclpy パッケージがインストールされている必要があります
- カメラデバイスが接続されている必要があります
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import argparse
import time

class CameraTestNode(Node):
    """
    カメラ画像を受信して処理するROS2ノード
    """
    
    def __init__(self, topic_name, depth_mode=False):
        """
        ノードの初期化と画像トピックのサブスクライバーを設定
        
        Args:
            topic_name (str): 購読する画像トピック名
            depth_mode (bool): 深度カメラモードを有効にするかどうか
        """
        super().__init__('camera_test_node')
        
        self.depth_mode = depth_mode
        self.bridge = CvBridge()
        
        # 画像トピックのサブスクライバーを作成
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10)  # QoSプロファイル
        
        self.get_logger().info(f'カメラ テストノードが起動しました。トピック: {topic_name}')
        self.get_logger().info(f'モード: {"深度" if depth_mode else "RGB"}')
        
        # 画像処理のパラメータ
        self.enable_edge_detection = False
        self.enable_face_detection = not depth_mode  # 深度モードでは顔検出を無効化
        
        # 顔検出のための分類器（RGBモードのみ）
        if self.enable_face_detection:
            # OpenCVの顔検出分類器を読み込む
            self.face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            )
        
        # 統計情報の初期化
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        
        # 最新のフレームを保存
        self.latest_frame = None

    def image_callback(self, msg):
        """
        画像メッセージを受信したときに呼び出されるコールバック関数
        
        Args:
            msg (Image): 受信した画像メッセージ
        """
        self.frame_count += 1
        
        try:
            # ROS画像メッセージをOpenCV形式に変換
            if self.depth_mode:
                # 深度画像の場合、16ビット画像として変換
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                # 深度画像を可視化用に正規化（表示用）
                # 0-10mの範囲を0-255にスケーリング
                depth_array = np.array(cv_image, dtype=np.float32)
                cv_image_normalized = np.clip(depth_array / 10.0 * 255, 0, 255).astype(np.uint8)
                
                # カラーマップを適用して視覚化
                cv_image_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
                display_image = cv_image_colormap
                
                # 深度統計情報を計算
                valid_depth = depth_array[depth_array > 0]
                if len(valid_depth) > 0:
                    min_depth = np.min(valid_depth)
                    max_depth = np.max(valid_depth)
                    avg_depth = np.mean(valid_depth)
                    self.get_logger().info(
                        f'深度範囲: 最小={min_depth:.2f}m, 最大={max_depth:.2f}m, 平均={avg_depth:.2f}m'
                    )
            else:
                # RGB画像の場合
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                display_image = cv_image.copy()
            
            # 画像の基本情報を表示
            height, width = cv_image.shape[:2]
            self.get_logger().info(f'フレーム #{self.frame_count} 受信: {width}x{height}')
            
            # エッジ検出（トグル可能）
            if self.enable_edge_detection and not self.depth_mode:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                display_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            # 顔検出（RGBモードのみ、トグル可能）
            if self.enable_face_detection and not self.depth_mode:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(
                    gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
                )
                
                # 検出された顔に矩形を描画
                for (x, y, w, h) in faces:
                    cv2.rectangle(display_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                if len(faces) > 0:
                    self.get_logger().info(f'  {len(faces)} 個の顔を検出しました')
            
            # FPSを計算（1秒ごと）
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 1.0:
                self.fps = self.frame_count / elapsed_time
                self.get_logger().info(f'FPS: {self.fps:.1f}')
                self.frame_count = 0
                self.start_time = time.time()
            
            # FPSを画像に表示
            cv2.putText(
                display_image, f'FPS: {self.fps:.1f}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
            )
            
            # 画像を表示
            cv2.imshow('MentorPi Camera Test', display_image)
            
            # キー入力を処理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # 'q'キーで終了
                self.get_logger().info('ユーザーによって終了されました')
                rclpy.shutdown()
            elif key == ord('e'):
                # 'e'キーでエッジ検出をトグル
                if not self.depth_mode:
                    self.enable_edge_detection = not self.enable_edge_detection
                    self.get_logger().info(f'エッジ検出: {"有効" if self.enable_edge_detection else "無効"}')
            elif key == ord('f'):
                # 'f'キーで顔検出をトグル
                if not self.depth_mode:
                    self.enable_face_detection = not self.enable_face_detection
                    self.get_logger().info(f'顔検出: {"有効" if self.enable_face_detection else "無効"}')
            
            # 最新のフレームを保存
            self.latest_frame = display_image
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge エラー: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'画像処理エラー: {str(e)}')

def main():
    """
    メイン関数
    """
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='MentorPi カメラ テストプログラム')
    parser.add_argument('--depth', action='store_true', help='深度カメラモードを有効にする')
    parser.add_argument('--topic', type=str, help='購読する画像トピック名')
    args = parser.parse_args()
    
    # デフォルトのトピック名
    if args.topic:
        topic_name = args.topic
    else:
        if args.depth:
            topic_name = '/ascamera/camera_publisher/depth/image'
        else:
            topic_name = '/ascamera/camera_publisher/rgb0/image'
    
    # ROS2の初期化
    rclpy.init()
    
    # カメラテストノードの作成
    camera_test_node = CameraTestNode(topic_name, args.depth)
    
    try:
        # ノードの実行
        rclpy.spin(camera_test_node)
    except KeyboardInterrupt:
        # Ctrl+Cで終了
        camera_test_node.get_logger().info('ユーザーによって終了されました')
    except Exception as e:
        # その他の例外
        camera_test_node.get_logger().error(f'エラーが発生しました: {str(e)}')
    finally:
        # ノードの破棄
        camera_test_node.destroy_node()
        # ROS2のシャットダウン
        rclpy.shutdown()
        # OpenCVのウィンドウを閉じる
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()