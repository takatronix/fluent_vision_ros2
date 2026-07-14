from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fv_soundboard'

launch_files = glob(os.path.join('launch', '*.launch.py'))
config_files = glob(os.path.join('config', '*.yaml'))
sound_files = glob(os.path.join('sounds', '*.wav'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'sounds'), sound_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fluent Robotics',
    maintainer_email='maintainer@example.com',
    description='状況イベント→サウンド+任意TTS+記録マーカーのサウンドボード (Event Bus v1.1)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fv_soundboard_node = fv_soundboard.soundboard_node:main',
        ],
    },
)
