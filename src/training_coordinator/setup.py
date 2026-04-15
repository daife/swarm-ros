from setuptools import find_packages, setup

package_name = "training_coordinator"

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
    description="Training coordinator node for reset workflow and episode stepping.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "training_coordinator_node = training_coordinator.training_coordinator_node:main",
        ],
    },
)
