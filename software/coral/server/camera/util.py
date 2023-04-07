import math
from typing import Tuple, List, Iterable

import cv2
import numpy as np


def crop_circle(
    img: np.ndarray, center_offset: Tuple[int, int], radius: int
) -> np.ndarray:
    """Crop an image to a circle."""

    # Mask circle
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    center = (
        img.shape[1] // 2 - center_offset[1],
        img.shape[0] // 2 - center_offset[0],
    )
    cv2.circle(mask, center, radius, color=(255, 255, 255), thickness=cv2.FILLED)
    img = cv2.bitwise_and(img, img, mask=mask)

    # Crop to the circle
    x, y, w, h = cv2.boundingRect(mask)
    return img[y : y + h, x : x + w]


def mask(
    frame: np.ndarray,
    lower_bound: Tuple[int, int, int],
    upper_bound: Tuple[int, int, int],
) -> np.ndarray:
    """Mask a frame."""

    return cv2.inRange(frame, lower_bound, upper_bound)


def close_mask(mask: np.ndarray, kernel_size: int, iterations: int) -> np.ndarray:
    """Close a mask."""

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)


def find_contours(mask: np.ndarray) -> Tuple[List[np.ndarray], np.ndarray]:
    """Find contours in a mask."""

    return cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


def look_through_contours(
    contours: List[np.ndarray], min_area: float, max_area: float
) -> Iterable[np.ndarray]:
    """
    Look through contours and return the ones that are within the given area range,
    in order of descending area.
    """

    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contours = filter(lambda cnt: min_area < cv2.contourArea(cnt) < max_area, contours)
    return list(contours)


def draw_mask(img: np.ndarray, mask: np.ndarray):
    """Annotate a mask directly on an image."""

    inv_mask = mask == 0
    img[inv_mask] = img[inv_mask] * 0.2


def draw_cross(
    img: np.ndarray, center: Tuple[int, int], color: Tuple[int, int, int]
) -> None:
    """Draw a cross directly on an image."""

    cv2.line(
        img,
        (center[0] - 10, center[1]),
        (center[0] + 10, center[1]),
        color,
        thickness=1,
    )
    cv2.line(
        img,
        (center[0], center[1] - 10),
        (center[0], center[1] + 10),
        color,
        thickness=1,
    )


"""
pixels |   cm
 28.88 |   3.25
 43.75 |   8.25
 57.98 |  13.25
 70.21 |  18.25
 83.10 |  23.25
 94.13 |  28.25
102.73 |  33.25
110.56 |  38.25
118.92 |  43.25
125.57 |  48.25
132.10 |  53.25
137.39 |  58.25
143.55 |  63.25
147.76 |  68.25
151.27 |  73.25
153.97 |  78.25
156.77 |  83.25
158.99 |  88.25
160.98 |  93.25
162.64 |  98.25
163.52 | 103.25

degree | MSE (cm^2)
   0   |     916.67
   1   |      77.76
   2   |      15.97
   3   |       5.09
   4   |       1.97
   5   |       0.67
   6   |       0.22 (used here)
   7   |       0.17
"""
_map_distance = np.poly1d(
    [
        3.41792074e-10,
        -1.82131076e-07,
        3.86478574e-05,
        -4.13076362e-03,
        2.33489935e-01,
        -6.22244528e00,
        6.43527942e01,
    ]
)

"""
degree | MSE (pixels^2)
   0   |        1669.11
   1   |         141.58
   2   |           2.30
   3   |           0.24 (used here)
   4   |           0.21
"""
_unmap_distance = np.poly1d(
    [6.66401621e-05, -2.50893393e-02, 3.28639544e00, 1.85942917e01]
)


def cartesian_to_polar(dx: float, dy: float) -> Tuple[float, float]:
    """
    Convert relative cartesian coordinates to polar coordinates from the "center" in
    (angle, distance).
    """

    # Compute relative distance to "center"
    # Compute raw angle
    angle = math.degrees(math.atan2(dy, dx))
    # Ensure 0º is pointing up and normalize angle to (-180º, 180º]
    angle = -angle + 90
    angle = angle - 360 if angle > 180 else angle

    # Compute ball distance
    distance = math.sqrt(dx**2 + dy**2)

    return angle, distance


def polar_to_cartesian(angle: float, distance: float) -> Tuple[float, float]:
    """
    Convert polar coordinates to relative cartesian coordinates from the "center" in
    (dx, dy).
    """

    dx = distance * math.sin(math.radians(angle))
    dy = distance * math.cos(math.radians(angle))

    return dx, dy


def map_pixels_to_cm(
    frame_shape: Tuple[float, float], x: float, y: float
) -> Tuple[float, float]:
    """
    Convert cartesian coordinates on the frame to cm coordinates from the edge of the
    robot in (angle, distance).
    """

    # Centralise
    dx = x - frame_shape[1] / 2
    dy = frame_shape[0] / 2 - y

    # Convert to polar
    angle, distance = cartesian_to_polar(dx, dy)

    # Map to cm
    distance = _map_distance(distance) if distance >= 0 else -_map_distance(-distance)

    return angle, distance


def map_cm_to_pixels(
    frame_shape: Tuple[float, float], angle: float, distance: float
) -> Tuple[float, float]:
    """
    Convert polar coordinates on the frame to pixels coordinates on the image frame in
    (x, y).
    """

    # Map to pixels
    distance = (
        _unmap_distance(distance) if distance >= 0 else -_unmap_distance(-distance)
    )

    # Convert to cartesian
    dx, dy = polar_to_cartesian(angle, distance)

    # Uncentralise
    x = dx + frame_shape[1] / 2
    y = frame_shape[0] / 2 - dy

    return x, y


def flatten_tuple(data: Tuple) -> Iterable:
    """Flatten a tuple."""

    if isinstance(data, tuple):
        for x in data:
            yield from flatten_tuple(x)
    else:
        yield data
