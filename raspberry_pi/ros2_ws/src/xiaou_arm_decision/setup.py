from setuptools import find_packages, setup


package_name = "xiaou_arm_decision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/transformer_decision.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XiaoU",
    maintainer_email="maintainer@example.com",
    description="ROS2 Transformer decision bridge",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "decision_node = xiaou_arm_decision.decision_node:main",
            "decision_observation_bridge = xiaou_arm_decision.decision_observation_bridge:main",
        ]
    },
)
