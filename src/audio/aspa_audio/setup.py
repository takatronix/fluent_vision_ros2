from glob import glob
import os

from setuptools import find_packages, setup


package_name = "aspa_audio"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rust"), glob("rust/Cargo.*")),
        (os.path.join("share", package_name, "rust", "src", "bin"), glob("rust/src/bin/*.rs")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASPA Team",
    maintainer_email="info@vegetalia.co.jp",
    description="CPAL ROS 2 audio I/O and single-stream GStreamer playback control",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "playback_controller = aspa_audio.playback_controller:main",
    ]},
)
