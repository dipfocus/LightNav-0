from setuptools import find_packages, setup

package_name = "vln_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/scripts", ["scripts/mjpg_zmq_sender.py", "scripts/zmq_camera_viewer.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="jianghaiping",
    author_email="jianghaiping@velabot.ai",
    maintainer="jianghaiping",
    maintainer_email="jianghaiping@velabot.ai",
    description="XRAW v2 ZMQ camera bridge for the LightNav ROS interface.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["zmq_camera = vln_camera.zmq_camera_node:main"]},
)
