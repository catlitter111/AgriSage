from setuptools import setup
import os
from glob import glob

package_name = 'bottle_detection_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加模型文件
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        # 添加配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # 添加启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='瓶子检测ROS2包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bottle_detector_node = bottle_detection_ros2.bottle_detector_node:main',
            'stereo_camera_node = bottle_detection_ros2.stereo_camera_node:main',
        ],
    },
)