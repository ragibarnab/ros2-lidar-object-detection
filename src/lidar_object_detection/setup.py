from setuptools import find_packages, setup
import glob
import os

package_name = 'lidar_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob.glob('data/*bin')),
        (os.path.join('share', package_name, 'weights'), ['weights/epoch_160.pth'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ragib Arnab',
    maintainer_email='rae3840924@gmail.com',
    description='Lidar Object Detection ROS 2 Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_object_detector_node = lidar_object_detection.lidar_object_detector_node:main',
            'lidar_publisher_node = lidar_object_detection.lidar_publisher_node:main'
        ],
    },
)
