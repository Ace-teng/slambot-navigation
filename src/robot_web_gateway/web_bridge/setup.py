import os
from glob import glob
from setuptools import find_packages, setup

package_name = "web_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="PenguinSoul",
    maintainer_email="1270161395@qq.com",
    description="Read-only ROS 2 data gateway for web clients",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["web_bridge = web_bridge.main:main"]},
)
