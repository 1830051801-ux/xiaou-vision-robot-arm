from setuptools import find_packages, setup


package_name = "xiaou_arm_hardware"

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
    description="Hardware readiness gate for the XiaoU CAN ros2_control plugin",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "hardware_readiness_node = xiaou_arm_hardware.hardware_readiness_node:main"
        ]
    },
)
