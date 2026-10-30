from glob import glob

from setuptools import setup

package_name = 'walker_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BoyaZhangNB',
    maintainer_email='harvardboya@gmail.com',
    description='Ethernet camera -> ROS 2 -> rosbag + network stream',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = walker_camera.camera_node:main',
            'stream_sender = walker_camera.stream_sender_node:main',
        ],
    },
)
