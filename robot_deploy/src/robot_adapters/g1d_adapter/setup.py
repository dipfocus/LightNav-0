from setuptools import find_packages, setup

package_name = "g1d_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="jianghaiping",
    author_email="jianghaiping@velabot.ai",
    maintainer="jianghaiping",
    maintainer_email="jianghaiping@velabot.ai",
    description="Unitree G1-D wheeled chassis adapter for LightNav.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "g1d_adapter = g1d_adapter.adapter_node:main",
            "g1d_odometry = g1d_adapter.odometry_node:main",
        ]
    },
)
