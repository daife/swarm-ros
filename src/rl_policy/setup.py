from setuptools import find_packages, setup

package_name = "rl_policy"

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
    description="RL policy node skeleton for train/eval mode switching.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rl_policy_node = rl_policy.rl_policy_node:main",
        ],
    },
)
