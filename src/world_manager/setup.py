from setuptools import find_packages, setup

package_name = "world_manager"

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
    description="World manager node for obstacle map reset and publish.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_manager_node = world_manager.world_manager_node:main",
        ],
    },
)
