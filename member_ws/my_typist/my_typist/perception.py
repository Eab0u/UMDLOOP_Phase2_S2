"""Camera-only panel pose and keyboard-grid registration.

The panel pose comes from the four public ArUco marker geometries.  The
keyboard placement is deliberately not assumed: the supplied photograph of
the Redragon key grid is matched against a fronto-parallel rectification of
the current camera image.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from my_typist.data import KEY_UNIT


RECTIFIED_PX_PER_M = 2000.0
GRID_WIDTH_U = 18.25
GRID_HEIGHT_U = 6.25
PANEL_WIDTH = 0.400
PANEL_HEIGHT = 0.175
MARKER_SIZE = 0.020
MARKER_CENTERS = {
    0: (0.016, 0.016),
    1: (0.384, 0.016),
    2: (0.384, 0.159),
    3: (0.016, 0.159),
}


@dataclass(frozen=True)
class BoardDetection:
    """One image's board estimate and keyboard origin."""

    rotation_camera_board: np.ndarray
    translation_camera_board: np.ndarray
    key_origin_board: np.ndarray
    marker_ids: tuple
    reprojection_error_px: float
    match_score: float
    debug_image: np.ndarray


def camera_pose_world(q):
    """Return ``(R_world_camera, t_world_camera)`` from measured joints."""
    theta0, theta1, theta2 = (float(v) for v in q[:3])
    c0, s0 = np.cos(theta0), np.sin(theta0)
    phi = theta1 + theta2
    cp, sp = np.cos(phi), np.sin(phi)
    x_head = np.array([cp * c0, cp * s0, sp])
    y_head = np.array([-s0, c0, 0.0])
    z_head = np.array([-sp * c0, -sp * s0, cp])
    rotation = np.column_stack((-y_head, -z_head, x_head))

    c1, s1 = np.cos(theta1), np.sin(theta1)
    translation = np.array([0.0, 0.0, 0.3])
    translation += 0.6 * np.array([c1 * c0, c1 * s0, s1])
    translation += 0.4 * x_head
    return rotation, translation


def marker_object_corners(marker_id):
    """Marker TL, TR, BR, BL in the documented board frame."""
    cx, cy = MARKER_CENTERS[int(marker_id)]
    half = MARKER_SIZE / 2.0
    return np.array(
        [
            [cx - half, cy - half, 0.0],
            [cx + half, cy - half, 0.0],
            [cx + half, cy + half, 0.0],
            [cx - half, cy + half, 0.0],
        ],
        dtype=np.float64,
    )


def _detect_markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return {}
    found = {}
    for corners_i, marker_id in zip(corners, np.asarray(ids).reshape(-1)):
        marker_id = int(marker_id)
        if marker_id in MARKER_CENTERS and marker_id not in found:
            found[marker_id] = np.asarray(corners_i, dtype=np.float64).reshape(4, 2)
    return found


def _rectify_panel(image, image_points, board_points):
    ppm = RECTIFIED_PX_PER_M
    destination = board_points[:, :2] * ppm - 0.5
    homography, _ = cv2.findHomography(image_points, destination, method=0)
    if homography is None:
        raise ValueError("could not compute panel homography")
    size = (int(round(PANEL_WIDTH * ppm)), int(round(PANEL_HEIGHT * ppm)))
    rectified = cv2.warpPerspective(image, homography, size)
    return rectified, homography


def _find_keyboard_origin(rectified, template):
    ppm = RECTIFIED_PX_PER_M
    grid_size = (
        int(round(GRID_WIDTH_U * KEY_UNIT * ppm)),
        int(round(GRID_HEIGHT_U * KEY_UNIT * ppm)),
    )
    if template is None or template.size == 0:
        raise ValueError("keyboard reference image is unavailable")
    interpolation = cv2.INTER_AREA if template.shape[1] >= grid_size[0] else cv2.INTER_LINEAR
    reference = cv2.resize(template, grid_size, interpolation=interpolation)
    panel_gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    reference_gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)

    # Gradients make the score depend on key outlines and legends rather than
    # the large, nearly uniform black panel surrounding the keyboard.
    panel_edges = cv2.Canny(panel_gray, 35, 100)
    reference_edges = cv2.Canny(reference_gray, 35, 100)
    scores = cv2.matchTemplate(panel_edges, reference_edges, cv2.TM_CCOEFF_NORMED)
    _, score, _, location = cv2.minMaxLoc(scores)
    origin = (np.asarray(location, dtype=np.float64) + 0.5) / ppm
    return origin, float(score), grid_size


def estimate_board(image, camera_matrix, distortion, template):
    """Estimate board pose (camera frame) and key-grid origin from one frame."""
    markers = _detect_markers(image)
    if len(markers) < 3:
        raise ValueError(f"need at least 3 panel markers; detected {sorted(markers)}")

    ids = tuple(sorted(markers))
    object_points = np.concatenate([marker_object_corners(i) for i in ids])
    image_points = np.concatenate([markers[i] for i in ids])
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise ValueError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    residual = projected.reshape(-1, 2) - image_points
    reprojection_error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))

    rectified, _ = _rectify_panel(image, image_points, object_points)
    origin, match_score, grid_size = _find_keyboard_origin(rectified, template)

    debug = image.copy()
    for marker_id in ids:
        polygon = np.rint(markers[marker_id]).astype(np.int32)
        cv2.polylines(debug, [polygon], True, (0, 255, 0), 2)
        anchor = tuple(polygon[0])
        cv2.putText(debug, str(marker_id), anchor, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Project the detected key-grid rectangle back onto the raw image.
    board_box = np.array(
        [
            origin,
            origin + [GRID_WIDTH_U * KEY_UNIT, 0.0],
            origin + [GRID_WIDTH_U * KEY_UNIT, GRID_HEIGHT_U * KEY_UNIT],
            origin + [0.0, GRID_HEIGHT_U * KEY_UNIT],
        ],
        dtype=np.float64,
    )
    board_box_3d = np.column_stack((board_box, np.zeros(4)))
    image_box, _ = cv2.projectPoints(board_box_3d, rvec, tvec, camera_matrix, distortion)
    image_box_int = np.rint(image_box.reshape(-1, 2)).astype(np.int32)
    cv2.polylines(debug, [image_box_int], True, (255, 180, 0), 2)
    cv2.putText(
        debug,
        f"pose {reprojection_error:.2f}px  keyboard {match_score:.2f}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 180, 0),
        2,
    )

    return BoardDetection(
        rotation_camera_board=rotation,
        translation_camera_board=tvec.reshape(3),
        key_origin_board=origin,
        marker_ids=ids,
        reprojection_error_px=reprojection_error,
        match_score=match_score,
        debug_image=debug,
    )


def load_keyboard_template():
    """Load the public keyboard photograph shipped with the challenge."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("autotype_sim")) / "assets" / "keyboard_grid.png"
        )
    except Exception:
        pass
    candidates.extend(
        [
            Path("/opt/autotype/install/autotype_sim/share/autotype_sim/assets/keyboard_grid.png"),
            Path(__file__).resolve().parents[3] / "assets" / "keyboard_grid.png",
        ]
    )
    for path in candidates:
        if path.is_file():
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is not None:
                return image
    raise FileNotFoundError("keyboard_grid.png was not found in the challenge installation")
