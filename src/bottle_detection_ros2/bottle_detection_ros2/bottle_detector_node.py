#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测ROS2节点
订阅图像话题，检测瓶子并发布检测结果
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# 导入自定义消息
from bottle_detection_msgs.msg import BottleDetection

# 导入检测器类
from bottle_detection_ros2.bottle_detector import BottleDetector

class BottleDetectorNode(Node):
    """瓶子检测ROS2节点"""

    def __init__(self):
        super().__init__('bottle_detector_node')
        
        # 声明参数
        self.declare_parameter('model_path', 'models/yolo11n.rknn')
        self.declare_parameter('model_size', [640, 640])
        self.declare_parameter('camera_topic', '/stereo_camera/left/image_raw')
        self.declare_parameter('detection_topic', '/bottle_detection/detections')
        self.declare_parameter('visualization_topic', '/bottle_detection/visualization')
        self.declare_parameter('depth_topic', '/stereo_camera/depth')
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        model_size = self.get_parameter('model_size').value
        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        visualization_topic = self.get_parameter('visualization_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        
        # 解析模型路径（如果是相对路径，转换为绝对路径）
        if not os.path.isabs(model_path):
            package_share_dir = get_package_share_directory('bottle_detection_ros2')
            model_path = os.path.join(package_share_dir, model_path)
        
        self.get_logger().info(f'使用模型: {model_path}')
        
        # 初始化瓶子检测器
        self.detector = BottleDetector(model_path, tuple(model_size))
        if not self.detector.load_model():
            self.get_logger().error('加载瓶子检测模型失败')
            raise RuntimeError('加载瓶子检测模型失败')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10)
        
        # 创建发布者
        self.detection_pub = self.create_publisher(
            BottleDetection,
            detection_topic,
            10)
            
        self.visualization_pub = self.create_publisher(
            Image,
            visualization_topic,
            10)
        
        # 存储最新的深度图像
        self.latest_depth_image = None
        
        self.get_logger().info('瓶子检测节点已初始化')
    
    def depth_callback(self, depth_msg):
        """深度图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'转换深度图像失败: {e}')
    
    def image_callback(self, img_msg):
        """图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # 检测瓶子
            bottle_detections = self.detector.detect(cv_image)
            
            # 创建检测结果消息
            detection_msg = BottleDetection()
            detection_msg.header = img_msg.header
            detection_msg.num_detections = len(bottle_detections)
            
            # 复制用于可视化的图像
            vis_image = cv_image.copy()
            
            # 如果检测到瓶子
            if bottle_detections:
                # 提取结果
                for left, top, right, bottom, score, cx, cy in bottle_detections:
                    # 添加到消息
                    detection_msg.bbox_x1.append(float(left))
                    detection_msg.bbox_y1.append(float(top))
                    detection_msg.bbox_x2.append(float(right))
                    detection_msg.bbox_y2.append(float(bottom))
                    detection_msg.scores.append(float(score))
                    detection_msg.center_x.append(float(cx))
                    detection_msg.center_y.append(float(cy))
                    
                    # 计算距离（如果有深度图像）
                    distance = None
                    has_distance = False
                    
                    if self.latest_depth_image is not None:
                        # 确保坐标在深度图像范围内
                        if (0 <= cy < self.latest_depth_image.shape[0] and 
                            0 <= cx < self.latest_depth_image.shape[1]):
                            # 获取深度值（这里假设深度图是单通道float32类型，单位是米）
                            # 注意：根据实际深度图格式调整此处
                            depth_value = self.latest_depth_image[int(cy), int(cx)]
                            
                            # 检查深度值是否有效
                            if depth_value > 0 and depth_value < 10.0:  # 10米作为最大合理距离
                                distance = float(depth_value)
                                has_distance = True
                    
                    detection_msg.distances.append(float(distance) if distance is not None else 0.0)
                    detection_msg.has_distance.append(has_distance)
                    
                    # 在可视化图像上绘制检测结果
                    self.detector.draw_detection(
                        vis_image, 
                        (left, top, right, bottom, score), 
                        distance
                    )
            
            # 发布检测结果
            self.detection_pub.publish(detection_msg)
            
            # 发布可视化结果
            visualization_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            visualization_msg.header = img_msg.header
            self.visualization_pub.publish(visualization_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'转换图像失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')
    
    def destroy_node(self):
        """节点销毁时释放资源"""
        self.detector.release_model()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    bottle_detector_node = BottleDetectorNode()
    
    try:
        rclpy.spin(bottle_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        bottle_detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()