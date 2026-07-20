from glob import glob
import os

from setuptools import find_packages, setup


package_name = "fv_soundboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "sounds"), glob("sounds/*.wav")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Fluent Robotics",
    maintainer_email="maintainer@example.com",
    description="Resolve FluentVision events into ASPA SYSTEM speech and cues",
    license="MIT",
    entry_points={"console_scripts": [
        "fv_soundboard_node = fv_soundboard.soundboard_node:main",
    ]},
)
