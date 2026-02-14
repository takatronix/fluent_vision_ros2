from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fv_pipeline_editor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install web assets
        ('share/' + package_name + '/web',
            glob('web/*')),
        # Install config
        ('share/' + package_name + '/config',
            glob('config/*')),
    ],
    install_requires=['setuptools', 'aiohttp', 'pyyaml'],
    zip_safe=True,
    maintainer='FluentVision',
    maintainer_email='dev@fluentvision.io',
    description='FluentVision Pipeline Editor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'editor_node = fv_pipeline_editor.editor_node:main',
        ],
    },
)
