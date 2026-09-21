"""Six-axis arm math and hardware-safety primitives.

This package is deliberately independent from ROS 2 so its kinematics and
safety rules can be tested on the development PC and on the Raspberry Pi.
"""

from .grasp_families import (
    GraspFamilyPreview,
    GraspFamilyRegistryError,
    build_grasp_family_preview,
    load_grasp_family_registry,
)
from .kinematics import (
    IKResult,
    fk_space,
    ik_space,
    ik_space_multistart,
    jacobian_space,
)
from .model import ArmModel, load_default_model
from .taught_grasp import (
    TaughtGraspPlan,
    TaughtGraspPlanningError,
    TaughtGraspStage,
    build_taught_side_grasp_candidates,
    build_taught_side_grasp_plan,
)
from .taught_waypoint_plan import (
    TaughtWaypointPlanningError,
    TaughtWaypointSegment,
    build_taught_waypoint_preview,
)
from .teach_registry import (
    ObjectTeachSpec,
    TeachRegistryError,
    load_object_teach_registry,
    resolve_object_teach_spec,
    validate_taught_object_record,
)
from .trajectory import JointLimits, TrajectoryPoint, plan_quintic_joint_trajectory

__all__ = [
    "ArmModel",
    "GraspFamilyPreview",
    "GraspFamilyRegistryError",
    "IKResult",
    "JointLimits",
    "ObjectTeachSpec",
    "TaughtGraspPlan",
    "TaughtGraspPlanningError",
    "TaughtGraspStage",
    "TaughtWaypointPlanningError",
    "TaughtWaypointSegment",
    "TeachRegistryError",
    "TrajectoryPoint",
    "build_grasp_family_preview",
    "build_taught_side_grasp_candidates",
    "build_taught_side_grasp_plan",
    "build_taught_waypoint_preview",
    "fk_space",
    "ik_space",
    "ik_space_multistart",
    "jacobian_space",
    "load_default_model",
    "load_grasp_family_registry",
    "load_object_teach_registry",
    "plan_quintic_joint_trajectory",
    "resolve_object_teach_spec",
    "validate_taught_object_record",
]
