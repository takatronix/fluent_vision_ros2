from setuptools import setup
import os
from glob import glob

package_name = 'fv_browser_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='Browser webcam bridge for FluentVision online demos',
    license='MIT',
    entry_points={
        'console_scripts': [
            'browser_camera_node = fv_browser_camera.browser_camera_node:main',
        ],
    },
)
