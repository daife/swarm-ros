from setuptools import find_packages, setup

package_name = "sim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Bridge Gazebo topics to unified /agent_* ROS topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_bridge_node = sim_bridge.sim_bridge_node:main",
        ],
    },
)
