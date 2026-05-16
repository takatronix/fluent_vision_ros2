from setuptools import setup
from glob import glob
import os

package_name = 'fv_apriltag'


def files_in(dirpath):
    return [f for f in glob(os.path.join(dirpath, '*')) if os.path.isfile(f)]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', files_in('launch')),
        ('share/' + package_name + '/config', files_in('config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='AprilTag detection + cube bundle pose estimation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_node = fv_apriltag.apriltag_node:main',
            'cube_estimator_node = fv_apriltag.cube_estimator_node:main',
        ],
    },
)
