import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ur5_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Kamel",
    maintainer_email="mkamel860@gmail.com",
    description="Vision-guided, collision-aware UR5e pick-and-place with MoveIt 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Nodes are registered here as they are implemented in later stages.
        ],
    },
)
