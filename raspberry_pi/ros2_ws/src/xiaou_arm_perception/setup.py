from setuptools import find_packages, setup


package_name = "xiaou_arm_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XiaoU",
    maintainer_email="maintainer@example.com",
    description="YOLO to base_link pose adapter",
    license="Proprietary",
    entry_points={"console_scripts": ["target_pose_node = xiaou_arm_perception.target_pose_node:main"]},
)
