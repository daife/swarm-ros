from setuptools import find_packages, setup

package_name = "agent_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "ament_index_python"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Agent manager node for reset orchestration and safe spawn checks.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agent_manager_node = agent_manager.agent_manager_node:main",
        ],
    },
)
