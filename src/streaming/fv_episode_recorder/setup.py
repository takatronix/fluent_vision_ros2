from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fv_episode_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'aiohttp',
        'pydantic',
        'python-ulid',
        'pyarrow',
    ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='FluentVision',
    maintainer_email='dev@fluentvision.io',
    description='FluentVision Episode Recorder — rosbag + camera mp4 storage with markers, retention, replay',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder_node = fv_episode_recorder.recorder_node:main',
            'video_anomaly_detector = fv_episode_recorder.video_anomaly_detector_node:main',
            'moss_realtime_adapter = fv_episode_recorder.moss_realtime_adapter_node:main',
            'moss_vl_realtime_runtime = fv_episode_recorder.moss_runtime:main',
        ],
    },
)
