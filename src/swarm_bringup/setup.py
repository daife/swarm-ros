import os
from glob import glob

from setuptools import find_packages, setup

package_name = "swarm_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Launch package for game_of_drones_rl skeleton.",
    license="Apache-2.0",
)
