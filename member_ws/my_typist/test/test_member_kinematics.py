"""Pure-Python checks for the member implementation (no simulator needed)."""

import math

import numpy as np

from my_typist.kinematics import ArmKinematics
from my_typist.planning import choose_typing_pose


def test_inverse_and_forward_arm_kinematics_round_trip():
    kinematics = ArmKinematics()
    target = (0.72, -0.08, 0.49)
    joints = kinematics.get_arm_joint_positions(*target)
    assert joints is not None
    np.testing.assert_allclose(
        kinematics.get_arm_location(*joints), target, atol=1e-10, rtol=0.0
    )


def test_stylus_inverse_points_at_target():
    kinematics = ArmKinematics()
    arm = kinematics.get_arm_joint_positions(0.78, 0.02, 0.47)
    target = np.array([1.02, -0.04, 0.41])
    pan, tilt = kinematics.get_stylus_joint_positions(*target, *arm)
    aim = np.asarray(kinematics.get_stylus_aim(*arm, pan, tilt))
    head = np.asarray(kinematics.get_arm_location(*arm))
    expected = target - head
    expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(aim, expected, atol=1e-10, rtol=0.0)


def test_planner_finds_one_pose_for_wide_key_span():
    board_origin = np.array([1.05, 0.18, 0.48])
    rotation_world_board = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    board_points = [(0.05, 0.04), (0.18, 0.08), (0.34, 0.12)]
    targets = [
        rotation_world_board @ np.array([x, y, 0.0]) + board_origin
        for x, y in board_points
    ]
    plan = choose_typing_pose(targets, [-1.0, 0.0, 0.0])
    kinematics = ArmKinematics()
    actual_head = kinematics.get_arm_location(*plan.arm_joints)
    np.testing.assert_allclose(actual_head, plan.head_position, atol=1e-10, rtol=0.0)
    for target in targets:
        distance = np.linalg.norm(target - np.asarray(plan.head_position))
        pan, tilt = kinematics.get_stylus_joint_positions(*target, *plan.arm_joints)
        assert 0.05 <= distance <= 0.35
        assert abs(pan) < math.radians(45.0)
        assert abs(tilt) < math.radians(35.0)
