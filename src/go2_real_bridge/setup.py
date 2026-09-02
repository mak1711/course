import os
from glob import glob

from setuptools import find_packages, setup

package_name = "go2_real_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kan",
    maintainer_email="mohammad.mk012@gmail.com",
    description="Bridges the real Go2's SDK topics (SportModeState, lidar) into what slam_toolbox/Nav2 expect (Odometry, TF, /scan) -- nothing upstream provides these.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_tf_bridge = go2_real_bridge.odom_tf_bridge:main",
        ],
    },
)
