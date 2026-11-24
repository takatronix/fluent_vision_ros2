"""Detector backends for :mod:`fv_apriltag` (pure Python, no ROS).

Two backends produce the same :class:`TagDetection` records so that
``apriltag_node`` is backend-agnostic:

``pupil``
    pupil-apriltags (AprilTag 3). Preferred where the wheel exists
    (x86_64 containers).
``opencv``
    ``cv2.aruco`` with the ``DICT_APRILTAG_*`` dictionaries plus
    ``solvePnP(IPPE_SQUARE)``. For hosts where pupil-apriltags cannot be
    installed (aarch64 containers). Nothing is picked silently: the node
    logs which backend runs, and ``backend: pupil`` fails loudly when the
    module is missing.

Both backends report AprilTag's conventions, so TF and downstream
consumers see no difference:

* ``corners``: ``p0 = (-1, +1)``, ``p1 = (+1, +1)``, ``p2 = (+1, -1)``,
  ``p3 = (-1, -1)`` in the tag frame, whose x points right and y points
  DOWN when the tag is viewed upright. In the image of an upright tag
  that is bottom-left, bottom-right, top-right, top-left.
* ``pose_R`` / ``pose_t``: map tag-frame points into the camera optical
  frame (x right, y down, z forward). The tag's z axis points into the
  tag, so an upright tag facing the camera has ``R = I``.
* ``homography``: maps tag coordinates ``(-1,-1) -> p0``, ``(1,-1) -> p1``,
  ``(1,1) -> p2``, ``(-1,1) -> p3`` (apriltag's ``det->H``; note the
  y-up convention of the homography, unlike the pose frame).

OpenCV's ``DICT_APRILTAG_36h11`` canonical marker image is the official
apriltag-imgs image rotated by 180 degrees (checked bit for bit over ids
0-115 and asserted in ``test/test_detector_backends.py``). ArUco lists
the corners clockwise from the top-left of *its* canonical orientation,
``[TL', TR', BR', BL']``, which on the real tag is ``[BR, BL, TL, TR]``.
Hence AprilTag order = ``aruco[[1, 0, 3, 2]]`` (:data:`ARUCO_TO_APRILTAG`).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


#: Index permutation turning an ArUco corner quad into AprilTag order.
ARUCO_TO_APRILTAG = (1, 0, 3, 2)

#: Tag-frame xy of the four corners in AprilTag order (unit half-edge).
#: Same order ``solvePnP(IPPE_SQUARE)`` requires for its object points.
TAG_CORNERS_XY = np.array(
    [[-1.0, 1.0], [1.0, 1.0], [1.0, -1.0], [-1.0, -1.0]], dtype=np.float64
)
#: Tag coordinates apriltag pairs with ``p0..p3`` when it fits ``det->H``.
HOMOGRAPHY_CORNERS_XY = np.array(
    [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]], dtype=np.float32
)

_OPENCV_FAMILIES = {
    'tag16h5': 'DICT_APRILTAG_16h5',
    'tag25h9': 'DICT_APRILTAG_25h9',
    'tag36h10': 'DICT_APRILTAG_36h10',
    'tag36h11': 'DICT_APRILTAG_36h11',
}
#: Bits AprilTag 3 corrects by default (``apriltag_detector_add_family``).
_APRILTAG_DEFAULT_BITS_CORRECTED = 2

BACKEND_CHOICES = ('auto', 'pupil', 'opencv')


class BackendUnavailable(RuntimeError):
    """Raised when the requested backend cannot run on this host."""


@dataclass
class TagDetection:
    """One decoded tag. Field names follow ``pupil_apriltags.Detection``."""

    tag_id: int
    hamming: int
    decision_margin: float
    center: np.ndarray        # (2,) pixels
    corners: np.ndarray       # (4, 2) pixels, AprilTag order
    homography: np.ndarray    # (3, 3)
    pose_R: Optional[np.ndarray]   # (3, 3) tag -> camera optical frame
    pose_t: Optional[np.ndarray]   # (3, 1) metres, tag -> camera optical frame


class PupilBackend:
    """pupil-apriltags (AprilTag 3) wrapper."""

    name = 'pupil'

    def __init__(self, family: str, nthreads: int = 2, quad_decimate: float = 2.0,
                 quad_sigma: float = 0.0, refine_edges: int = 1,
                 decode_sharpening: float = 0.25) -> None:
        try:
            from pupil_apriltags import Detector
        except ImportError as exc:  # pragma: no cover - depends on the host
            raise BackendUnavailable(
                f'pupil_apriltags not available: {exc}. Install via '
                '`pip3 install pupil-apriltags` (no wheel for aarch64 — '
                'use backend: opencv there).'
            ) from exc
        self._detector = Detector(
            families=family,
            nthreads=int(nthreads),
            quad_decimate=float(quad_decimate),
            quad_sigma=float(quad_sigma),
            refine_edges=int(refine_edges),
            decode_sharpening=float(decode_sharpening),
        )

    def describe(self) -> str:
        return 'pupil-apriltags (AprilTag 3)'

    def detect(self, gray: np.ndarray, camera_params: Sequence[float],
               tag_size: float) -> List[TagDetection]:
        raw = self._detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=tuple(float(v) for v in camera_params),
            tag_size=float(tag_size),
        )
        out: List[TagDetection] = []
        for det in raw:
            out.append(TagDetection(
                tag_id=int(det.tag_id),
                hamming=int(det.hamming),
                decision_margin=float(det.decision_margin),
                center=np.asarray(det.center, dtype=np.float64).reshape(2),
                corners=np.asarray(det.corners, dtype=np.float64).reshape(4, 2),
                homography=np.asarray(det.homography, dtype=np.float64).reshape(3, 3),
                pose_R=None if det.pose_R is None else np.asarray(det.pose_R, dtype=np.float64).reshape(3, 3),
                pose_t=None if det.pose_t is None else np.asarray(det.pose_t, dtype=np.float64).reshape(3, 1),
            ))
        return out


def _aruco_detect_fn(family: str, quad_decimate: float, quad_sigma: float
                     ) -> Tuple[Callable[[np.ndarray], tuple], int]:
    """Build a ``gray -> (corners, ids, rejected)`` callable for the host's
    OpenCV (4.6 ``detectMarkers`` API or 4.7+ ``ArucoDetector``)."""
    aruco = getattr(cv2, 'aruco', None)
    if aruco is None:
        raise BackendUnavailable('cv2.aruco is missing (opencv-contrib needed)')
    try:
        dictionary_id = getattr(aruco, _OPENCV_FAMILIES[family])
    except KeyError as exc:
        raise BackendUnavailable(
            f'family {family!r} has no OpenCV ArUco dictionary '
            f'(supported: {sorted(_OPENCV_FAMILIES)})'
        ) from exc

    def _setup(params, dictionary) -> None:
        # AprilTag 2 quad extraction + refinement inside ArUco: the only mode
        # that honours the quad_decimate / quad_sigma knobs of the pupil path.
        params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        params.aprilTagQuadDecimate = float(quad_decimate)
        params.aprilTagQuadSigma = float(quad_sigma)
        # tag36h11 has a 1-cell black border.
        params.markerBorderBits = 1
        # Correct at most the bits AprilTag 3 corrects by default, so a
        # noisy quad is rejected the same way on both backends.
        max_bits = int(dictionary.maxCorrectionBits)
        if max_bits > 0:
            params.errorCorrectionRate = min(
                float(params.errorCorrectionRate),
                _APRILTAG_DEFAULT_BITS_CORRECTED / max_bits,
            )

    if hasattr(aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        params = aruco.DetectorParameters()
        _setup(params, dictionary)
        detector = aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers, int(dictionary.maxCorrectionBits)
    dictionary = aruco.Dictionary_get(dictionary_id)   # OpenCV 4.6 and older
    params = aruco.DetectorParameters_create()
    _setup(params, dictionary)
    return (lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params),
            int(dictionary.maxCorrectionBits))


def _sample(gray: np.ndarray, H: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Bilinear samples of ``gray`` at tag coordinates ``uv`` (N, 2) via ``H``.
    Points outside the image come back as NaN."""
    pts = np.column_stack([uv, np.ones(len(uv))]) @ H.T
    xy = pts[:, :2] / pts[:, 2:3]
    h, w = gray.shape[:2]
    out = np.full(len(uv), np.nan, dtype=np.float64)
    inside = (xy[:, 0] >= 0) & (xy[:, 0] <= w - 1) & (xy[:, 1] >= 0) & (xy[:, 1] <= h - 1)
    if inside.any():
        sampled = cv2.remap(
            gray, xy[inside, 0].astype(np.float32).reshape(-1, 1),
            xy[inside, 1].astype(np.float32).reshape(-1, 1),
            cv2.INTER_LINEAR,
        )
        out[inside] = sampled.reshape(-1)
    return out


def decision_margin_from_image(gray: np.ndarray, H: np.ndarray,
                               data_cells: int = 6) -> float:
    """Approximate AprilTag's ``decision_margin`` for a decoded quad.

    AprilTag reports the mean distance of each data bit's intensity from the
    local black/white decision threshold (grey levels, 0-255). Here the
    threshold is the midpoint between the black border cells and the white
    quiet-ring cells (those inside the image), sampled through the quad's
    homography ``H`` (tag coordinates [-1, 1] span the black square).
    """
    n = data_cells + 2
    centres = -1.0 + (2.0 * np.arange(n) + 1.0) / n
    uu, vv = np.meshgrid(centres, centres)
    is_border = (uu == centres[0]) | (uu == centres[-1]) | (vv == centres[0]) | (vv == centres[-1])
    grid = np.column_stack([uu.ravel(), vv.ravel()])
    values = _sample(gray, H, grid)
    border = values[is_border.ravel()]
    data = values[~is_border.ravel()]
    ring_c = 1.0 + 1.0 / n
    ring = np.array([[u, -ring_c] for u in centres] + [[u, ring_c] for u in centres]
                    + [[-ring_c, v] for v in centres] + [[ring_c, v] for v in centres])
    white = _sample(gray, H, ring)
    white = white[np.isfinite(white)]
    data = data[np.isfinite(data)]
    border = border[np.isfinite(border)]
    if data.size == 0 or border.size == 0:
        return 0.0
    black_level = float(np.mean(border))
    white_level = float(np.mean(white)) if white.size else float(np.max(data))
    threshold = 0.5 * (black_level + white_level)
    return float(np.mean(np.abs(data - threshold)))


class OpenCvArucoBackend:
    """``cv2.aruco`` + ``solvePnP(IPPE_SQUARE)`` in AprilTag conventions."""

    name = 'opencv'

    def __init__(self, family: str, nthreads: int = 2, quad_decimate: float = 2.0,
                 quad_sigma: float = 0.0, refine_edges: int = 1,
                 decode_sharpening: float = 0.25) -> None:
        self.family = family
        self._detect, self._max_correction_bits = _aruco_detect_fn(
            family, quad_decimate, quad_sigma)
        self._ignored = {'nthreads': nthreads, 'refine_edges': refine_edges,
                         'decode_sharpening': decode_sharpening}

    def describe(self) -> str:
        return (f'OpenCV {cv2.__version__} ArUco {_OPENCV_FAMILIES[self.family]} '
                f'+ solvePnP(IPPE_SQUARE); not applied: {self._ignored}; '
                f'hamming is not reported (-1), decision_margin is '
                f'approximated from the quad')

    def detect(self, gray: np.ndarray, camera_params: Sequence[float],
               tag_size: float) -> List[TagDetection]:
        quads, ids, _rejected = self._detect(gray)
        if ids is None or len(ids) == 0:
            return []
        fx, fy, cx, cy = (float(v) for v in camera_params)
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        object_points = np.column_stack([
            TAG_CORNERS_XY * (float(tag_size) / 2.0), np.zeros(4)])
        out: List[TagDetection] = []
        for quad, tag_id in zip(quads, np.asarray(ids).reshape(-1)):
            corners = np.asarray(quad, dtype=np.float64).reshape(4, 2)[list(ARUCO_TO_APRILTAG)]
            H = cv2.getPerspectiveTransform(
                HOMOGRAPHY_CORNERS_XY, corners.astype(np.float32)).astype(np.float64)
            centre_h = H @ np.array([0.0, 0.0, 1.0])
            centre = centre_h[:2] / centre_h[2]
            ok, rvec, tvec = cv2.solvePnP(
                object_points, corners, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            R, _ = cv2.Rodrigues(rvec)
            out.append(TagDetection(
                tag_id=int(tag_id),
                hamming=-1,
                decision_margin=decision_margin_from_image(gray, H),
                center=centre,
                corners=corners,
                homography=H,
                pose_R=np.asarray(R, dtype=np.float64).reshape(3, 3),
                pose_t=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
            ))
        return out


def create_backend(name: str, family: str, **detector_kwargs):
    """Instantiate ``name`` (``auto`` / ``pupil`` / ``opencv``).

    ``auto`` takes pupil when importable, else OpenCV. Returns the backend
    plus a one-line reason for the choice (the node logs it)."""
    if name not in BACKEND_CHOICES:
        raise BackendUnavailable(f'unknown backend {name!r} (choices: {BACKEND_CHOICES})')
    if name == 'pupil':
        return PupilBackend(family, **detector_kwargs), 'requested'
    if name == 'opencv':
        return OpenCvArucoBackend(family, **detector_kwargs), 'requested'
    try:
        return PupilBackend(family, **detector_kwargs), 'auto: pupil_apriltags importable'
    except BackendUnavailable as exc:
        reason = str(exc).split('.')[0]
        return OpenCvArucoBackend(family, **detector_kwargs), f'auto: {reason}'
