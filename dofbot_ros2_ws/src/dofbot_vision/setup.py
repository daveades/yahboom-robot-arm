from setuptools import setup
import os
from glob import glob

package_name = 'dofbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('lib/' + package_name, [package_name + '/__init__.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Computer Vision for DOFBOT SE',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_sorter = dofbot_vision.color_sorter:main',
            'yolo_detector = dofbot_vision.yolo_detector:main',
            'pick_from_detections = dofbot_vision.pick_from_detections:main',
        ],
    },
)
