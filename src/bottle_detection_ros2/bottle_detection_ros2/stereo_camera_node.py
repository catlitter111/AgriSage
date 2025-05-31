#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机ROS2节点
处理双目相机的捕获、校正和深度计算，并发布相关图像话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# 导入双目相机类
from bottle_detection_ros2.stereo_camera import StereoCamera

class StereoCameraNode(Node):
    """双目相机ROS2节点"""

    def __init__(self):
        super().__init__('stereo_camera_node')
        
        # 声明参数
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('camera_params_file', 'config/camera_params.xls')
        self.declare_parameter('publish_rate', 30)  # Hz
        
        # 获取参数
        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.frame_rate = self.get_parameter('frame_rate').value
        camera_params_file = self.get_parameter('camera_params_file').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 解析相机参数文件路径
        if not os.path.isabs(camera_params_file):
            package_share_dir = get_package_share_directory('bottle_detection_ros2')
            camera_params_file = os.path.join(package_share_dir, camera_params_file)
        
        # 初始化双目相机
        self.stereo_camera = StereoCamera(camera_id, width, height)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建发布者
        self.left_raw_pub = self.create_publisher(
            Image,
            '/stereo_camera/left/image_raw',
            10)
            
        self.right_raw_pub = self.create_publisher(
            Image,
            '/stereo_camera/right/image_raw',
            10)
            
        self.left_rect_pub = self.create_publisher(
            Image,
            '/stereo_camera/left/image_rect',
            10)
            
        self.right_rect_pub = self.create_publisher(
            Image,
            '/stereo_camera/right/image_rect',
            10)
            
        self.disparity_pub = self.create_publisher(
            Image,
            '/stereo_camera/disparity',
            10)
            
        self.depth_pub = self.create_publisher(
            Image,
            '/stereo_camera/depth',
            10)
        
        # 加载相机参数
        if os.path.exists(camera_params_file):
            self.get_logger().info(f'从文件加载相机参数: {camera_params_file}')
            self.stereo_camera.load_camera_params(camera_params_file)
        else:
            self.get_logger().info('使用默认相机参数')
            self.stereo_camera.load_camera_params()
        
        # 设置双目校正参数
        self.stereo_camera.setup_stereo_rectification()
        
        # 打开相机
        if not self.stereo_camera.open_camera():
            self.get_logger().error('无法打开双目相机')
            raise RuntimeError('无法打开双目相机')
        
        # 创建定时器
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info('双目相机节点已初始化')
    
    def timer_callback(self):
        """定时器回调函数，捕获和处理图像"""
        # 捕获一帧图像
        frame_left, frame_right = self.stereo_camera.capture_frame()
        
        if frame_left is None or frame_right is None:
            self.get_logger().warn('无法接收帧')
            return
        
        try:
            # 获取当前时间戳
            current_time = self.get_clock().now().to_msg()
            
            # 发布原始图像
            left_raw_msg = self.bridge.cv2_to_imgmsg(frame_left, "bgr8")
            left_raw_msg.header.stamp = current_time
            left_raw_msg.header.frame_id = "camera_left_optical_frame"
            self.left_raw_pub.publish(left_raw_msg)
            
            right_raw_msg = self.bridge.cv2_to_imgmsg(frame_right, "bgr8")
            right_raw_msg.header.stamp = current_time
            right_raw_msg.header.frame_id = "camera_right_optical_frame"
            self.right_raw_pub.publish(right_raw_msg)
            
            # 校正图像
            frame_left_rectified, img_left_rectified, img_right_rectified = self.stereo_camera.rectify_stereo_images(frame_left, frame_right)
            
            # 发布校正后的图像
            left_rect_msg = self.bridge.cv2_to_imgmsg(frame_left_rectified, "bgr8")
            left_rect_msg.header.stamp = current_time
            left_rect_msg.header.frame_id = "camera_left_optical_frame"
            self.left_rect_pub.publish(left_rect_msg)
            
            right_rect_msg = self.bridge.cv2_to_imgmsg(img_right_rectified, "mono8")
            right_rect_msg.header.stamp = current_time
            right_rect_msg.header.frame_id = "camera_right_optical_frame"
            self.right_rect_pub.publish(right_rect_msg)
            
            # 计算视差图
            disparity, disp_normalized = self.stereo_camera.compute_disparity(img_left_rectified, img_right_rectified)
            
            # 发布视差图
            disparity_msg = self.bridge.cv2_to_imgmsg(disp_normalized, "mono8")
            disparity_msg.header.stamp = current_time
            disparity_msg.header.frame_id = "camera_left_optical_frame"
            self.disparity_pub.publish(disparity_msg)
            
            # 计算三维坐标
            threeD = self.stereo_camera.compute_3d_points(disparity)
            
            # 创建深度图
            # 从三维坐标中提取Z值，并转换为米
            depth = np.zeros_like(disparity, dtype=np.float32)
            if threeD is not None:
                # 提取Z值（深度），转换为米并处理无效值
                z = threeD[:, :, 2]
                z_valid = np.where((z > 0) & (z < 10000), z / 1000.0, 0)  # 将毫米转换为米，限制范围
                depth = z_valid.astype(np.float32)
            
            # 发布深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = "camera_left_optical_frame"
            self.depth_pub.publish(depth_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'转换图像失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')
    
    def destroy_node(self):
        """节点销毁时释放资源"""
        self.stereo_camera.close_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    stereo_camera_node = StereoCameraNode()
    
    try:
        rclpy.spin(stereo_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        stereo_camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()