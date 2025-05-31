#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测系统启动文件
启动双目相机和瓶子检测节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('bottle_detection_ros2')
    
    # 定义启动参数
    camera_id = LaunchConfiguration('camera_id')
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='相机设备ID'
    )
    
    model_path = LaunchConfiguration('model_path')
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/yolo11n.rknn',
        description='RKNN模型路径(相对于包共享目录)'
    )
    
    camera_params_file = LaunchConfiguration('camera_params_file')
    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value='config/camera_params.xls',
        description='相机参数文件路径(相对于包共享目录)'
    )
    
    # 双目相机节点
    stereo_camera_node = Node(
        package='bottle_detection_ros2',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        parameters=[
            {
                'camera_id': camera_id,
                'width': 1280,
                'height': 480,
                'frame_rate': 15.0,
                'camera_params_file': camera_params_file,
                'publish_rate': 15  # Hz
            }
        ],
        output='screen'
    )
    
    # 瓶子检测节点
    bottle_detector_node = Node(
        package='bottle_detection_ros2',
        executable='bottle_detector_node',
        name='bottle_detector_node',
        parameters=[
            {
                'model_path': model_path,
                'model_size': [640, 640],
                'camera_topic': '/stereo_camera/left/image_rect',
                'detection_topic': '/bottle_detection/detections',
                'visualization_topic': '/bottle_detection/visualization',
                'depth_topic': '/stereo_camera/depth'
            }
        ],
        output='screen'
    )
    
    # 创建启动描述
    return LaunchDescription([
        camera_id_arg,
        model_path_arg,
        camera_params_file_arg,
        stereo_camera_node,
        bottle_detector_node
    ])