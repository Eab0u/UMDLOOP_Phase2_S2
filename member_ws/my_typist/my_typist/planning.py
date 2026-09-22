"""Typing-pose search, kept independent from perception and ROS control."""

from dataclasses import dataclass
import math

import numpy as np

from my_typist.data import (
    HEAD_PAN_LIMIT,
    HEAD_TILT_LIMIT,
    STYLUS_MAX_REACH,
    STYLUS_MIN_REACH,
)
from my_typist.kinematics import ArmKinematics


JOINT_MIN = np.radians([-120.0, -30.0, -140.0])
JOINT_MAX = np.radians([120.0, 100.0, 0.0])


@dataclass(frozen=True)
class TypingPlan:
    arm_joints: tuple
    head_position: tuple
    key_targets: tuple
    worst_head_fraction: float


def _head_angles(kinematics, arm_joints, head_position, target):
    return kinematics.get_stylus_joint_positions(
        *target, *arm_joints
    )


def choose_typing_pose(key_targets, board_normal_toward_arm):
    """Search for one fixed arm pose from which every requested key is safe."""
    targets = np.asarray(key_targets, dtype=float).reshape(-1, 3)
    normal = np.asarray(board_normal_toward_arm, dtype=float)
    normal /= np.linalg.norm(normal)
    centre = np.mean(targets, axis=0)
    kin = ArmKinematics()
    best = None

    world_up = np.array([0.0, 0.0, 1.0])
    horizontal = np.cross(world_up, normal)
    if np.linalg.norm(horizontal) < 1e-6:
        horizontal = np.array([0.0, 1.0, 0.0])
    horizontal /= np.linalg.norm(horizontal)

    # Search along the panel normal and a small world-Z correction.  The
    # normal carries yaw/tilt, while the correction helps avoid shoulder and
    # elbow limits at the extremes of the episode sampling volume.
    for distance in np.linspace(0.16, 0.30, 15):
        for side_offset in np.linspace(-0.10, 0.10, 9):
            for dz in np.linspace(-0.08, 0.08, 9):
                head = (
                    centre
                    + distance * normal
                    + side_offset * horizontal
                    + np.array([0.0, 0.0, dz])
                )
                arm = kin.get_arm_joint_positions(*head)
                if arm is None:
                    continue
                arm = np.asarray(arm)
                if np.any(arm < JOINT_MIN + 0.02) or np.any(
                    arm > JOINT_MAX - 0.02
                ):
                    continue

                head_angles = []
                ranges = []
                valid = True
                for target in targets:
                    angles = _head_angles(kin, arm, head, target)
                    rng = float(np.linalg.norm(target - head))
                    direction = (target - head) / rng
                    incidence = math.acos(
                        float(np.clip(-np.dot(direction, normal), -1.0, 1.0))
                    )
                    if (
                        rng < STYLUS_MIN_REACH + 0.01
                        or rng > STYLUS_MAX_REACH - 0.01
                        or abs(angles[0]) > HEAD_PAN_LIMIT - 0.03
                        or abs(angles[1]) > HEAD_TILT_LIMIT - 0.03
                        or incidence > math.radians(50.0)
                    ):
                        valid = False
                        break
                    head_angles.append(angles)
                    ranges.append(rng)
                if not valid:
                    continue

                head_fraction = max(
                    max(abs(a[0]) / HEAD_PAN_LIMIT, abs(a[1]) / HEAD_TILT_LIMIT)
                    for a in head_angles
                )
                range_penalty = max(abs(r - 0.22) for r in ranges)
                movement_penalty = 0.03 * float(np.linalg.norm(arm))
                cost = head_fraction + 2.0 * range_penalty + movement_penalty
                if best is None or cost < best[0]:
                    best = (cost, arm, head, head_fraction)

    if best is None:
        raise ValueError("no common safe typing pose exists for the launch key")
    _, arm, head, head_fraction = best
    return TypingPlan(
        arm_joints=tuple(float(v) for v in arm),
        head_position=tuple(float(v) for v in head),
        key_targets=tuple(tuple(float(v) for v in target) for target in targets),
        worst_head_fraction=float(head_fraction),
    )
