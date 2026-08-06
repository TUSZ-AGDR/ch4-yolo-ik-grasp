from setuptools import find_packages, setup
from glob import glob

package_name = 'yolo_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLO visual detection for Gazebo block grid simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector = yolo_vision.detector_node:main',
        ],
    },
)
