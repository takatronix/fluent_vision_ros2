from setuptools import find_packages, setup


package_name = "fv_machine_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fluent Vision Team",
    maintainer_email="info@vegetalia.co.jp",
    description="Stationary-bracket hand-eye collection and offline solver",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fv_handeye_collect = fv_machine_calibration.collector_node:main",
            "fv_handeye_solve = fv_machine_calibration.cli:main",
        ],
    },
)
