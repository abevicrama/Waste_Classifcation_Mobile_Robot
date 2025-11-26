"""QR code detection and decoding utilities.

This module provides functions to detect and decode QR codes from an image
and a small CLI that prints decoded text and bounding boxes when run on an
input image file.
"""

import sys
from typing import List, Tuple, Dict

import cv2


def detect_and_decode_qr(image: cv2.Mat) -> List[Dict[str, object]]:
    """Detect and decode QR codes in the given image.

    Returns a list of detections. Each detection is a dict with keys:
      - 'data': the decoded string ('' if detection failed to decode)
      - 'bbox': (x, y, w, h) integer bounding box
      - 'points': the numpy array of polygon points (4x2) when available

    Works with both single and multiple QR codes (uses detectAndDecodeMulti
    when available, otherwise falls back to detectAndDecode).
    """
    qr = cv2.QRCodeDetector()

    # Prefer detectAndDecodeMulti (multiple QR codes)
    try:
        decoded_info, points, _ = qr.detectAndDecodeMulti(image)
    except Exception:
        # OpenCV builds may differ; fall back to single-code API
        data, points = qr.detectAndDecode(image)[0:2]
        if points is None:
            return []
        # points returned for single detection is shape (4,2)
        # Normalize to lists for consistent return type
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        x, y = int(x_coords.min()), int(y_coords.min())
        w, h = int(x_coords.max() - x), int(y_coords.max() - y)
        return [{"data": data, "bbox": (x, y, w, h), "points": points}]

    if points is None:
        return []

    detections: List[Dict[str, object]] = []
    # decoded_info is a list of strings, points is a list/array of polygons
    for i, pts in enumerate(points):
        info = decoded_info[i] if decoded_info is not None and i < len(decoded_info) else ""
        pts_arr = pts.reshape((-1, 2)) if hasattr(pts, 'reshape') else pts
        x_coords = pts_arr[:, 0]
        y_coords = pts_arr[:, 1]
        x, y = int(x_coords.min()), int(y_coords.min())
        w, h = int(x_coords.max() - x), int(y_coords.max() - y)
        detections.append({"data": info, "bbox": (x, y, w, h), "points": pts_arr})

    return detections


def _read_image(path: str) -> cv2.Mat:
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return img


def _print_detections(detections: List[Dict[str, object]]) -> None:
    if not detections:
        print("No QR codes detected")
        return
    for i, d in enumerate(detections, start=1):
        print(f"Detection {i}:")
        print(f"  data: {d['data']}")
        print(f"  bbox: {d['bbox']}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python qr.py <image_path>")
        sys.exit(1)
    path = sys.argv[1]
    img = _read_image(path)
    dets = detect_and_decode_qr(img)
    _print_detections(dets)