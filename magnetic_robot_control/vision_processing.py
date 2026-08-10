from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class CircleDetectionResult:
    detected: bool
    centroid_x: float | None
    centroid_y: float | None
    center_x: float
    center_y: float
    offset_x: float | None
    offset_y: float | None
    deviation: float | None
    mask_area: int
    contour_area: float


def detect_black_ring_center(rgb_frame, min_area=250.0):
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("Circle detection requires opencv-python and numpy.") from error

    if rgb_frame is None:
        return None, None

    image = rgb_frame
    height, width = image.shape[:2]
    center_x = width / 2.0
    center_y = height / 2.0

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return CircleDetectionResult(
            detected=False,
            centroid_x=None,
            centroid_y=None,
            center_x=center_x,
            center_y=center_y,
            offset_x=None,
            offset_y=None,
            deviation=None,
            mask_area=int(mask.sum() / 255),
            contour_area=0.0,
        ), mask

    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    if contour_area < min_area:
        return CircleDetectionResult(
            detected=False,
            centroid_x=None,
            centroid_y=None,
            center_x=center_x,
            center_y=center_y,
            offset_x=None,
            offset_y=None,
            deviation=None,
            mask_area=int(mask.sum() / 255),
            contour_area=contour_area,
        ), mask

    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        return CircleDetectionResult(
            detected=False,
            centroid_x=None,
            centroid_y=None,
            center_x=center_x,
            center_y=center_y,
            offset_x=None,
            offset_y=None,
            deviation=None,
            mask_area=int(mask.sum() / 255),
            contour_area=contour_area,
        ), mask

    centroid_x = moments["m10"] / moments["m00"]
    centroid_y = moments["m01"] / moments["m00"]
    offset_x = centroid_x - center_x
    offset_y = centroid_y - center_y
    deviation = hypot(offset_x, offset_y)

    return CircleDetectionResult(
        detected=True,
        centroid_x=float(centroid_x),
        centroid_y=float(centroid_y),
        center_x=center_x,
        center_y=center_y,
        offset_x=float(offset_x),
        offset_y=float(offset_y),
        deviation=float(deviation),
        mask_area=int(mask.sum() / 255),
        contour_area=contour_area,
    ), mask
