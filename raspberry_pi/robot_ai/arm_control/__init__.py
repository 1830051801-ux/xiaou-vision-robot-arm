"""Six-axis arm math and hardware-safety primitives.

This package is deliberately independent from ROS 2 so its kinematics and
safety rules can be tested on the development PC and on the Raspberry Pi.
"""

from .kinematics import IKResult, fk_space, ik_space, ik_space_multistart, jacobian_space
from .model import ArmModel, load_default_model
from .trajectory import JointLimits, TrajectoryPoint, plan_quintic_joint_trajectory
from .taught_grasp import TaughtGraspPlan, TaughtGraspPlanningError, TaughtGraspStage, build_taught_side_grasp_plan
from .grasp_families import (
    GraspFamilyPreview,
    GraspFamilyRegistryError,
    build_grasp_family_preview,
    load_grasp_family_registry,
)
from .teach_registry import (
    ObjectTeachSpec,
    TeachRegistryError,
    load_object_teach_registry,
    resolve_object_teach_spec,
    validate_taught_object_record,
)
from .taught_waypoint_plan import (
    TaughtWaypointPlanningError,
    TaughtWaypointSegment,
    build_taught_waypoint_preview,
)

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
    "TaughtGraspPlan",
    "TaughtGraspPlanningError",
    "TaughtGraspStage",
    "build_taught_side_grasp_plan",
    "GraspFamilyPreview",
    "GraspFamilyRegistryError",
    "build_grasp_family_preview",
    "load_grasp_family_registry",
    "ObjectTeachSpec",
    "TeachRegistryError",
    "load_object_teach_registry",
    "resolve_object_teach_spec",
    "validate_taught_object_record",
    "TaughtWaypointPlanningError",
    "TaughtWaypointSegment",
    "build_taught_waypoint_preview",
]
