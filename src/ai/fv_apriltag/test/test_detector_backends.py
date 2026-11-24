"""Backend parity tests for fv_apriltag.detector_backends (cv2 + numpy only).

What must hold:
  - OpenCV's DICT_APRILTAG_36h11 marker image is the official apriltag-imgs
    image rotated by 180 degrees (that is what the corner permutation rests on)
  - the OpenCV backend returns corners in AprilTag order (bottom-left,
    bottom-right, top-right, top-left of an upright tag), the pose in the
    tag -> camera-optical convention (upright tag facing the camera: R = I,
    z into the tag), translation scaling with tag_size, and a decision margin
    that falls when the tag's contrast falls
  - when pupil-apriltags is importable (x86_64 containers) both backends agree
    on the same synthetic image; where it is not, backend 'pupil' fails loudly
    and 'auto' takes OpenCV
"""
import math

import cv2
import numpy as np
import pytest

from fv_apriltag.detector_backends import (
    ARUCO_TO_APRILTAG,
    BackendUnavailable,
    OpenCvArucoBackend,
    TAG_CORNERS_XY,
    create_backend,
)

# tag36h11 codewords (36 data bits, row-major from the top-left data cell,
# 1 = white) read from the official apriltag-imgs PNGs. 0 and 1 are the first
# two entries of the family; 204/221 are the first and last container ids of
# the 200 band in config/tag_registry.yaml.
TAG36H11_CODES = {0: 0xD5D628584, 1: 0xD97F18B49, 204: 0xE6E819E94, 221: 0xC601896D1}
CAMERA = (600.0, 600.0, 320.0, 240.0)       # fx, fy, cx, cy
IMAGE_SHAPE = (480, 640)
TAG_SIZE = 0.060                             # black edge [m]


def tag_bitmap(code: int) -> np.ndarray:
    """10x10 uint8 image: white quiet ring, black border, 6x6 data (1 = white)."""
    img = np.zeros((10, 10), dtype=np.uint8)
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 255
    for k in range(36):
        if (code >> (35 - k)) & 1:
            img[2 + k // 6, 2 + k % 6] = 255
    return img


def _aruco_marker(tag_id: int) -> np.ndarray:
    aruco = cv2.aruco
    if hasattr(aruco, 'getPredefinedDictionary'):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    else:
        dictionary = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, 'generateImageMarker'):
        return aruco.generateImageMarker(dictionary, tag_id, 8, borderBits=1)
    return aruco.drawMarker(dictionary, tag_id, 8, borderBits=1)


def _rotation(axis, degrees: float) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(axis, dtype=np.float64) * math.radians(degrees))
    return R


def _project(points_tag: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy = CAMERA
    cam = points_tag @ R.T + t.reshape(1, 3)
    return np.column_stack([fx * cam[:, 0] / cam[:, 2] + cx, fy * cam[:, 1] / cam[:, 2] + cy])


def render(tag_id: int, R: np.ndarray, t: np.ndarray, tag_size: float = TAG_SIZE,
           black: int = 0, white: int = 255, flip_bits=(), gradient: float = 0.0) -> np.ndarray:
    """Upright apriltag-imgs artwork of ``tag_id`` placed at (R, t) in the camera
    frame, projected onto a mid-grey canvas. Tag x right, y down, z into the tag.
    ``flip_bits`` inverts data cells (row, col); ``gradient`` darkens the artwork
    linearly from left to right by that fraction (uneven lighting)."""
    bitmap = tag_bitmap(TAG36H11_CODES[tag_id]).astype(np.float64) / 255.0
    for row, col in flip_bits:
        bitmap[2 + row, 2 + col] = 1.0 - bitmap[2 + row, 2 + col]
    art = (black + bitmap * (white - black)).astype(np.float64)
    art = cv2.resize(art, (200, 200), interpolation=cv2.INTER_NEAREST)
    if gradient:
        art = art * (1.0 - gradient * np.linspace(0.0, 1.0, 200)[None, :])
    art = np.clip(art, 0, 255).astype(np.uint8)
    half = tag_size * 5.0 / 8.0                      # 10 cells incl. the quiet ring
    tag_xy = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
    dst = _project(np.column_stack([tag_xy, np.zeros(4)]), R, t).astype(np.float32)
    src = np.array([[0, 0], [200, 0], [200, 200], [0, 200]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(art, H, (IMAGE_SHAPE[1], IMAGE_SHAPE[0]),
                               flags=cv2.INTER_LINEAR, borderValue=128)


def _angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    cos = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


@pytest.fixture(scope='module')
def backend():
    return OpenCvArucoBackend('tag36h11', quad_decimate=1.0)


@pytest.mark.parametrize('tag_id', sorted(TAG36H11_CODES))
def test_opencv_marker_is_the_apriltag_artwork_rotated_180(tag_id):
    official = tag_bitmap(TAG36H11_CODES[tag_id])[1:9, 1:9]       # black border + data
    drawn = _aruco_marker(tag_id)
    assert drawn.shape == (8, 8)
    assert np.array_equal(np.rot90(drawn, 2) > 127, official > 127)
    assert not np.array_equal(drawn > 127, official > 127)         # i.e. not the identity


@pytest.mark.parametrize('tag_id', [0, 204, 221])
def test_upright_tag_facing_the_camera_has_identity_rotation(backend, tag_id):
    t = np.array([0.02, -0.01, 0.40])
    dets = backend.detect(render(tag_id, np.eye(3), t), CAMERA, TAG_SIZE)
    assert [d.tag_id for d in dets] == [tag_id]
    det = dets[0]
    assert _angle_deg(det.pose_R, np.eye(3)) < 1.0
    assert np.allclose(det.pose_t.reshape(3), t, atol=0.003)
    # corner order: bottom-left, bottom-right, top-right, top-left in the image
    bl, br, tr, tl = det.corners
    assert bl[0] < br[0] and tl[0] < tr[0]
    assert bl[1] > tl[1] and br[1] > tr[1]
    assert det.hamming == 0 and det.decision_margin > 60.0


def test_tilted_tag_pose_and_corners_match_the_projection(backend):
    R = _rotation([0, 0, 1], 35.0) @ _rotation([1, 0, 0], -30.0) @ _rotation([0, 1, 0], 20.0)
    t = np.array([-0.06, 0.04, 0.45])
    dets = backend.detect(render(204, R, t), CAMERA, TAG_SIZE)
    assert len(dets) == 1 and dets[0].tag_id == 204
    det = dets[0]
    expected = _project(np.column_stack([TAG_CORNERS_XY * TAG_SIZE / 2, np.zeros(4)]), R, t)
    assert np.allclose(det.corners, expected, atol=1.0), det.corners - expected
    assert np.allclose(det.center, _project(np.zeros((1, 3)), R, t)[0], atol=1.0)
    assert _angle_deg(det.pose_R, R) < 1.5
    assert np.allclose(det.pose_t.reshape(3), t, atol=0.004)
    # homography pairs (-1,-1) with p0 like apriltag's det->H
    p0 = det.homography @ np.array([-1.0, -1.0, 1.0])
    assert np.allclose(p0[:2] / p0[2], det.corners[0], atol=1e-6)


def test_translation_scales_with_tag_size_and_rotation_does_not(backend):
    R = _rotation([1, 0, 0], -20.0)
    t = np.array([0.0, 0.0, 0.50])
    gray = render(204, R, t)
    small = backend.detect(gray, CAMERA, TAG_SIZE)[0]
    big = backend.detect(gray, CAMERA, 2 * TAG_SIZE)[0]
    assert np.allclose(big.pose_t, 2 * small.pose_t, rtol=0.01)
    assert _angle_deg(big.pose_R, small.pose_R) < 0.5


def test_decision_margin_falls_with_contrast(backend):
    t = np.array([0.0, 0.0, 0.40])
    crisp = backend.detect(render(204, np.eye(3), t), CAMERA, TAG_SIZE)[0]
    washed = backend.detect(render(204, np.eye(3), t, black=90, white=170), CAMERA, TAG_SIZE)[0]
    assert crisp.decision_margin > 100.0
    assert 10.0 < washed.decision_margin < crisp.decision_margin / 2.0


def test_hamming_counts_the_bits_that_differ_from_the_codeword(backend):
    t = np.array([0.0, 0.0, 0.40])
    clean = backend.detect(render(204, np.eye(3), t), CAMERA, TAG_SIZE)[0]
    one = backend.detect(render(204, np.eye(3), t, flip_bits=[(2, 3)]), CAMERA, TAG_SIZE)[0]
    two = backend.detect(render(204, np.eye(3), t, flip_bits=[(0, 0), (5, 5)]), CAMERA, TAG_SIZE)[0]
    assert (clean.tag_id, one.tag_id, two.tag_id) == (204, 204, 204)
    assert (clean.hamming, one.hamming, two.hamming) == (0, 1, 2)
    # more errors than AprilTag 3 corrects by default are not decoded
    assert backend.detect(render(204, np.eye(3), t, flip_bits=[(0, 0), (2, 3), (5, 5)]), CAMERA, TAG_SIZE) == []


def test_only_verified_families_are_offered_on_opencv():
    for family in ('tag16h5', 'tag25h9', 'tag36h10', 'tag41h12'):
        with pytest.raises(BackendUnavailable):
            OpenCvArucoBackend(family)


def test_corner_permutation_is_the_documented_one():
    assert ARUCO_TO_APRILTAG == (1, 0, 3, 2)
    assert np.array_equal(TAG_CORNERS_XY, [[-1, 1], [1, 1], [1, -1], [-1, -1]])


def test_unknown_family_and_backend_fail_loudly():
    with pytest.raises(BackendUnavailable):
        OpenCvArucoBackend('tag41h12')
    with pytest.raises(BackendUnavailable):
        create_backend('fast', 'tag36h11')


def test_auto_picks_a_backend_and_says_why():
    backend_obj, reason = create_backend('auto', 'tag36h11', quad_decimate=1.0)
    assert backend_obj.name in ('pupil', 'opencv')
    assert reason.startswith('auto:')
    try:
        import pupil_apriltags  # noqa: F401
    except ImportError:
        assert backend_obj.name == 'opencv'
        with pytest.raises(BackendUnavailable):
            create_backend('pupil', 'tag36h11')
    else:
        assert backend_obj.name == 'pupil'


@pytest.mark.parametrize('case', [
    dict(R=_rotation([0, 0, 1], 25.0) @ _rotation([1, 0, 0], -35.0), t=[0.03, -0.02, 0.42]),
    dict(R=np.eye(3), t=[0.0, 0.0, 0.30], black=40, white=200),                   # low contrast
    dict(R=_rotation([0, 1, 0], 30.0), t=[-0.05, 0.04, 0.50], gradient=0.5),      # uneven lighting
    dict(R=_rotation([0, 0, 1], -60.0), t=[0.13, 0.09, 0.40]),                    # off-centre, rolled
    dict(R=np.eye(3), t=[0.0, 0.0, 0.40], flip_bits=[(1, 4)]),                   # one corrected bit
])
def test_opencv_agrees_with_pupil_when_both_are_available(case):
    pytest.importorskip('pupil_apriltags')
    pupil, _ = create_backend('pupil', 'tag36h11', quad_decimate=1.0)
    opencv = OpenCvArucoBackend('tag36h11', quad_decimate=1.0)
    case = dict(case)
    R, t = case.pop('R'), np.array(case.pop('t'))
    gray = render(204, R, t, **case)
    (a,) = pupil.detect(gray, CAMERA, TAG_SIZE)
    (b,) = opencv.detect(gray, CAMERA, TAG_SIZE)
    assert a.tag_id == b.tag_id == 204
    assert a.hamming == b.hamming
    assert np.allclose(a.corners, b.corners, atol=0.75)
    assert _angle_deg(a.pose_R, b.pose_R) < 1.0
    assert np.allclose(a.pose_t, b.pose_t, atol=0.003)
    # same decode metric (quad_decode's grey models), so the 50.0 gate means the same thing
    assert b.decision_margin == pytest.approx(a.decision_margin, rel=0.15, abs=3.0)
