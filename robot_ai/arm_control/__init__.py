"""Six-axis arm math and hardware-safety primitives.

This package is deliberately independent from ROS 2 so its kinematics and
safety rules can be tested on the development PC and on the Raspberry Pi.
"""

from .kinematics import IKResult, fk_space, ik_space, ik_space_multistart, jacobian_space
from .model import ArmModel, load_default_model
from .trajectory import JointLimits, TrajectoryPoint, plan_quintic_joint_trajectory

__all__ = [
    "ArmModel",
    "IKResult",
    "JointLimits",
    "TrajectoryPoint",
    "fk_space",
    "ik_space",
    "ik_space_multistart",
    "jacobian_space",
    "load_default_model",
    "plan_quintic_joint_trajectory",
]
