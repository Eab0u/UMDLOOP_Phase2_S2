"""Rendered-frame integration check for the member perception pipeline."""

import numpy as np

from autotype_sim.core.board import BoardGeometry, sample_board_pose
from autotype_sim.core.config import ArmConfig, CameraConfig
from autotype_sim.core.keymap import generate_tkl
from autotype_sim.core.kinematics import forward_kinematics
from autotype_sim.core.renderer import build_texture, render
from autotype_sim.testing import default_geometry
from my_typist.perception import estimate_board, load_keyboard_template


def test_panel_pose_and_keyboard_origin_from_rendered_camera_frame():
    raw_geometry = default_geometry()
    geometry = BoardGeometry.from_dict(raw_geometry)
    keymap = generate_tkl(raw_geometry)
    arm = ArmConfig()
    camera = CameraConfig()
    home = forward_kinematics(arm, arm.q_home)
    template = load_keyboard_template()
    texture = build_texture(geometry, keymap, photo=template)
    board_pose = sample_board_pose(7, geometry, arm, camera)
    image = render(
        camera,
        home.R_cam,
        home.t_cam,
        board_pose,
        texture,
        geometry,
    )

    detection = estimate_board(image, camera.K, np.zeros(5), template)
    expected_rotation = home.R_cam.T @ board_pose.R
    expected_translation = home.R_cam.T @ (board_pose.t - home.t_cam)

    np.testing.assert_allclose(
        detection.key_origin_board,
        geometry.key_area_origin,
        atol=0.001,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        detection.translation_camera_board,
        expected_translation,
        atol=0.002,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        detection.rotation_camera_board,
        expected_rotation,
        atol=0.005,
        rtol=0.0,
    )
    assert detection.reprojection_error_px < 2.0
    assert detection.match_score > 0.20
