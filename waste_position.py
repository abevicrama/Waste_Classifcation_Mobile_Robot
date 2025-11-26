"""Capture one frame from the camera and run a YOLO model on it.

This script provides a helper `run_model_on_one_frame()` which:
- acquires a single frame (Picamera2 on Raspberry Pi, OpenCV on other machines),
- runs an Ultralytics YOLO model located at `models/best2.pt`,
- prints detected bounding boxes and saves a visualization to `last_detection.jpg`.

The function avoids importing hardware-specific libraries at module import time so
the file can be imported on development machines that don't have Picamera2.
"""

from typing import List, Tuple, Dict, Any


def _capture_frame() -> Any:
    """Return a single frame as a NumPy array (BGR for OpenCV).

    Tries Picamera2 first, falls back to OpenCV's VideoCapture(0).
    """
    # For local webcam testing we currently disable Picamera2 and use OpenCV.
    # If you later want to re-enable Picamera2, uncomment the block below and
    # remove the OpenCV-only code.
    #
    # --- Picamera2 block (commented out) ---
    # try:
    #     from picamera2 import Picamera2
    #
    #     picam2 = Picamera2()
    #     preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    #     picam2.configure(preview_config)
    #     picam2.start()
    #     frame = picam2.capture_array()
    #     picam2.stop()
    #     try:
    #         import cv2
    #         frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #     except Exception:
    #         pass
    #     return frame
    # except Exception:
    #     pass
    # --- end Picamera2 block ---

    # Use OpenCV webcam for testing
    try:
        import cv2

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError("OpenCV VideoCapture could not open the camera")
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("OpenCV failed to read a frame from the camera")
        return frame
    except Exception as exc:
        raise RuntimeError(f"Failed to capture frame with OpenCV webcam: {exc}")


def run_model_on_one_frame(model_path: str = "models/best2.pt") -> List[Dict[str, Any]]:
    """Load YOLO model, run inference on one captured frame, return bounding boxes.

    Returns a list of detections where each detection is a dict containing
    'xyxy' (xmin, ymin, xmax, ymax), 'conf' (confidence) and 'cls' (class id).
    Also writes a visualization image to `last_detection.jpg` if any detections are found.
    """
    # Lazy import ultralytics to avoid import errors on machines without it
    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise RuntimeError(f"Failed to import ultralytics YOLO: {exc}\nInstall with: pip install ultralytics")

    # load model
    model = YOLO(model_path)

    # capture one frame
    frame = _capture_frame()
    if frame is None:
        raise RuntimeError("No frame captured")

    # Run inference. Ultralytics accepts NumPy arrays directly.
    results = model(frame)
    for res in results:
        res.show()
    detections: List[Dict[str, Any]] = []
    
    

    # `results` can be an object or a list depending on ultralytics version/interface
    iterable = results if isinstance(results, (list, tuple)) else [results]

    for res in iterable:
        # Each res should have `.boxes` with `.xyxy`, `.conf`, `.cls`
        boxes = getattr(res, "boxes", None)
        if boxes is None:
            continue

        # Attempt to get numpy arrays from the ultralytics boxes
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy()
        except Exception:
            # Fallback if cpu()/numpy() not available
            try:
                xyxy = boxes.xyxy.numpy()
                confs = boxes.conf.numpy()
                classes = boxes.cls.numpy()
            except Exception:
                # As a last resort, try to read attributes directly
                xyxy = getattr(boxes, "xyxy", None)
                confs = getattr(boxes, "conf", None)
                classes = getattr(boxes, "cls", None)

        if xyxy is None:
            continue

        for i, box in enumerate(xyxy):
            det = {
                "xyxy": [float(x) for x in box],
                "conf": float(confs[i]) if confs is not None else None,
                "cls": int(classes[i]) if classes is not None else None,
            }
            detections.append(det)

        # Save a visualized image if possible
        try:
            vis = res.plot()
            import cv2

            cv2.imwrite("last_detection.jpg", vis)
        except Exception:
            # ignore visualization errors
            pass
    
    # Build a list of detected class names (uses the model's names mapping).
    try:
        class_names = [model.names[int(d["cls"]) ] if d.get("cls") is not None else None for d in detections]
    except Exception:
        # If mapping fails for any reason, fall back to an empty list.
        class_names = []

    # Print the detected class names as requested.
    print("Detected class names:", class_names)

    return detections


if __name__ == "__main__":
    # quick smoke test when run directly
    try:
        dets = run_model_on_one_frame()
        if not dets:
            print("No detections")
        else:
            print("Detections:")
            for d in dets:
                print(d)
    except Exception as e:
        print(f"Error: {e}")