"""Detector backends for :mod:`fv_apriltag` (pure Python, no ROS).

Two backends produce the same :class:`TagDetection` records so that
``apriltag_node`` is backend-agnostic:

``pupil``
    pupil-apriltags (AprilTag 3). Preferred where the wheel exists
    (x86_64 containers).
``opencv``
    ``cv2.aruco`` with the ``DICT_APRILTAG_36h11`` dictionary plus
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
* ``decision_margin`` / ``hamming``: the OpenCV backend re-reads the data
  bits through the homography the way ``quad_decode()`` in apriltag.c
  does (linear grey models fitted on the black border and the white quiet
  ring, per-bit threshold at their midpoint, margin = min of the mean
  white and mean black distances) and counts the bits that differ from
  the dictionary codeword. So ``min_decision_margin`` keeps its meaning
  across backends (parity asserted in ``test/test_detector_backends.py``
  where pupil is importable).

OpenCV's ``DICT_APRILTAG_36h11`` canonical marker image is the official
apriltag-imgs image rotated by 180 degrees (checked bit for bit over ids
0-115 and asserted in the tests). ArUco lists the corners clockwise from
the top-left of *its* canonical orientation, ``[TL', TR', BR', BL']``,
which on the real tag is ``[BR, BL, TL, TR]``. Hence AprilTag order =
``aruco[[1, 0, 3, 2]]`` (:data:`ARUCO_TO_APRILTAG`). Only tag36h11 is
offered on the OpenCV backend because that relation was verified for
tag36h11 only.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

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

#: family -> (OpenCV dictionary attribute, data cells per side). Only the
#: families whose canonical-orientation relation to apriltag-imgs was
#: verified are listed (see module docstring).
_OPENCV_FAMILIES = {
    'tag36h11': ('DICT_APRILTAG_36h11', 6),
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


class _ArucoApi:
    """The host's ArUco entry points (OpenCV 4.6 module functions or 4.7+ classes)."""

    def __init__(self, family: str, quad_decimate: float, quad_sigma: float) -> None:
        aruco = getattr(cv2, 'aruco', None)
        if aruco is None:
            raise BackendUnavailable('cv2.aruco is missing (opencv-contrib needed)')
        try:
            dict_attr, self.data_cells = _OPENCV_FAMILIES[family]
        except KeyError as exc:
            raise BackendUnavailable(
                f'family {family!r} is not offered on the OpenCV backend '
                f'(verified: {sorted(_OPENCV_FAMILIES)})'
            ) from exc
        try:
            dictionary_id = getattr(aruco, dict_attr)
            if hasattr(aruco, 'ArucoDetector'):                  # OpenCV >= 4.7
                self.dictionary = aruco.getPredefinedDictionary(dictionary_id)
                params = aruco.DetectorParameters()
                self._setup(aruco, params)
                detector = aruco.ArucoDetector(self.dictionary, params)
                self.detect = detector.detectMarkers
            else:                                                # OpenCV 4.6 and older
                self.dictionary = aruco.Dictionary_get(dictionary_id)
                params = aruco.DetectorParameters_create()
                self._setup(aruco, params)
                dictionary = self.dictionary
                self.detect = lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)
            self._marker_image = getattr(aruco, 'generateImageMarker', None) or aruco.drawMarker
            self._codewords = {}
            self.marker_size = int(self.dictionary.markerSize)
            if self.marker_size != self.data_cells:
                raise BackendUnavailable(
                    f'{dict_attr}: markerSize {self.marker_size} != {self.data_cells} data cells')
            self.codeword_bits(0)                                # exercise the bit decoder once
        except BackendUnavailable:
            raise
        except (AttributeError, cv2.error, TypeError, ValueError) as exc:
            raise BackendUnavailable(
                f'this OpenCV build ({cv2.__version__}) lacks the ArUco AprilTag '
                f'support fv_apriltag needs: {exc}'
            ) from exc

    def _setup(self, aruco, params) -> None:
        # AprilTag 2 quad extraction + refinement inside ArUco: the only mode
        # that honours the quad_decimate / quad_sigma knobs of the pupil path.
        params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        params.aprilTagQuadDecimate = float(self._quad_decimate)
        params.aprilTagQuadSigma = float(self._quad_sigma)
        params.markerBorderBits = 1                              # tag36h11: 1-cell black border
        # Correct exactly the bits AprilTag 3 corrects by default (2), so a
        # noisy quad is accepted / rejected the same way on both backends.
        # OpenCV's predefined APRILTAG dictionaries ship with
        # maxCorrectionBits = 0 (4.6 and 4.11 alike), i.e. no correction at
        # all; the attribute is writable and ArUco uses
        # int(errorCorrectionRate * maxCorrectionBits).
        self.dictionary.maxCorrectionBits = _APRILTAG_DEFAULT_BITS_CORRECTED
        params.errorCorrectionRate = 1.0

    def __new__(cls, family: str, quad_decimate: float, quad_sigma: float):
        self = super().__new__(cls)
        self._quad_decimate = quad_decimate
        self._quad_sigma = quad_sigma
        return self

    def codeword_bits(self, tag_id: int) -> np.ndarray:
        """Data bits of ``tag_id`` as apriltag-imgs shows them (top row first, 1 = white).

        Taken from the dictionary's own marker image (the same source the
        tests compare with apriltag-imgs) rather than from ``bytesList``,
        whose packing is not the same across OpenCV versions.
        """
        if tag_id not in self._codewords:
            side = self.marker_size + 2                      # 1-cell black border
            image = np.asarray(self._marker_image(self.dictionary, int(tag_id), side, 1))
            if image.shape != (side, side):
                raise ValueError(f'unexpected marker image shape {image.shape}')
            bits = (image[1:-1, 1:-1] > 127).astype(np.uint8)
            # OpenCV's canonical image is the apriltag-imgs image rotated by 180°.
            self._codewords[tag_id] = np.rot90(bits, 2)
        return self._codewords[tag_id]


def _sample(gray: np.ndarray, H: np.ndarray, uv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Bilinear samples of ``gray`` at tag coordinates ``uv`` (N, 2) via ``H``,
    plus the pixel positions. Points outside the image come back as NaN."""
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
    return out, xy


class _GrayModel:
    """Least-squares plane ``v = a*u + b*v' + c`` over tag coordinates (apriltag's graymodel)."""

    def __init__(self, uv: np.ndarray, values: np.ndarray) -> None:
        ok = np.isfinite(values)
        self.n = int(ok.sum())
        if self.n >= 3:
            A = np.column_stack([uv[ok], np.ones(self.n)])
            self.coef, *_ = np.linalg.lstsq(A, values[ok], rcond=None)
        elif self.n > 0:
            self.coef = np.array([0.0, 0.0, float(np.mean(values[ok]))])
        else:
            self.coef = None

    def at(self, uv: np.ndarray) -> np.ndarray:
        return np.column_stack([uv, np.ones(len(uv))]) @ self.coef


def read_bits(gray: np.ndarray, H: np.ndarray, data_cells: int) -> Tuple[np.ndarray, float]:
    """Decode the data cells through ``H`` like apriltag.c ``quad_decode()``.

    Returns the bit matrix (top row first, 1 = white) and the decision
    margin: grey models are fitted on the black border cells and the white
    quiet-ring cells (those inside the image); each data bit is compared
    with the midpoint of the two models at its own position; the margin is
    the smaller of the mean white distance and the mean black distance
    (counts start at 1 as in apriltag.c). Tag coordinates: [-1, 1] spans
    the black square, +v is up (apriltag's homography convention).
    """
    n = data_cells + 2
    centres = -1.0 + (2.0 * np.arange(n) + 1.0) / n
    ring = 1.0 + 1.0 / n
    border_uv = np.array([[u, v] for u in centres for v in (centres[0], centres[-1])]
                         + [[u, v] for v in centres[1:-1] for u in (centres[0], centres[-1])])
    ring_uv = np.array([[u, v] for u in centres for v in (-ring, ring)]
                       + [[u, v] for v in centres for u in (-ring, ring)])
    black = _GrayModel(border_uv, _sample(gray, H, border_uv)[0])
    white = _GrayModel(ring_uv, _sample(gray, H, ring_uv)[0])
    # data cells, top row first: v runs from +1 down to -1
    rows = centres[1:-1][::-1]
    cols = centres[1:-1]
    data_uv = np.array([[u, v] for v in rows for u in cols])
    values, _ = _sample(gray, H, data_uv)
    bits = np.zeros(len(data_uv), dtype=np.uint8)
    if black.coef is None or white.coef is None or not np.isfinite(values).all():
        return bits.reshape(data_cells, data_cells), 0.0
    threshold = 0.5 * (black.at(data_uv) + white.at(data_uv))
    diff = values - threshold
    is_white = diff > 0
    bits[is_white] = 1
    white_score = float(diff[is_white].sum()) / (1 + int(is_white.sum()))
    black_score = float((-diff[~is_white]).sum()) / (1 + int((~is_white).sum()))
    return bits.reshape(data_cells, data_cells), float(min(white_score, black_score))


class OpenCvArucoBackend:
    """``cv2.aruco`` + ``solvePnP(IPPE_SQUARE)`` in AprilTag conventions."""

    name = 'opencv'

    def __init__(self, family: str, nthreads: int = 2, quad_decimate: float = 2.0,
                 quad_sigma: float = 0.0, refine_edges: int = 1,
                 decode_sharpening: float = 0.25) -> None:
        self.family = family
        self._api = _ArucoApi(family, quad_decimate, quad_sigma)
        self._ignored = {'nthreads': nthreads, 'refine_edges': refine_edges,
                         'decode_sharpening': decode_sharpening}

    def describe(self) -> str:
        return (f'OpenCV {cv2.__version__} ArUco {_OPENCV_FAMILIES[self.family][0]} '
                f'+ solvePnP(IPPE_SQUARE); bits re-read through the homography for '
                f'decision_margin / hamming; not applied: {self._ignored}')

    def detect(self, gray: np.ndarray, camera_params: Sequence[float],
               tag_size: float) -> List[TagDetection]:
        quads, ids, _rejected = self._api.detect(gray)
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
            bits, margin = read_bits(gray, H, self._api.data_cells)
            hamming = int(np.count_nonzero(bits != self._api.codeword_bits(int(tag_id))))
            out.append(TagDetection(
                tag_id=int(tag_id),
                hamming=hamming,
                decision_margin=margin,
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
