from setuptools import find_packages, setup


package_name = "xiaou_arm_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/desktop_scene.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XiaoU",
    maintainer_email="maintainer@example.com",
    description="Offline ROS2 desktop-object task scene",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "desktop_scene_node = xiaou_arm_simulation.desktop_scene_node:main",
        ]
    },
)
