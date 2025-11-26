import cv2
import numpy as np
import os
from typing import List, Dict, Any

# --- CONFIGURATION ---
# IMPORTANT: You must list your class names here in the order they were trained.
# If you only have one class (waste), just leave it as is.
CLASS_NAMES = ["waste"] 
MODEL_PATH = "models/best2.onnx"
INPUT_SIZE = (640, 640) # Must match the size used during export (yolo export ... imgsz=160)
CONF_THRESHOLD = 0.25   # Minimum confidence to detect
NMS_THRESHOLD = 0.45    # Lower value = less overlapping boxes

def _capture_frame() -> Any:
    """Return a single frame as a NumPy array."""
    # --- Option 1: Use Camera (Uncomment when ready for robot) ---
    # try:
    #     cap = cv2.VideoCapture(0)
    #     if not cap.isOpened():
    #         raise RuntimeError("Camera not found")
    #     ret, frame = cap.read()
    #     cap.release()
    #     if not ret:
    #         raise RuntimeError("Failed to read frame")
    #     return frame
    # except Exception as e:
    #     raise RuntimeError(f"Camera error: {e}")

    # --- Option 2: Use Test Image (Current Setup) ---
    img_path = "testing_image/test_02.jpeg"
    if not os.path.exists(img_path):
        raise RuntimeError(f"Test image not found at: {img_path}")
    
    frame = cv2.imread(img_path)
    if frame is None:
        raise RuntimeError("Failed to load test image")
    return frame

def run_model_on_one_frame(model_path: str = MODEL_PATH) -> List[Dict[str, Any]]:
    # 1. Load Model
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found at {model_path}")
    
    net = cv2.dnn.readNetFromONNX(model_path)

    # 2. Get Frame
    frame = _capture_frame()
    img_height, img_width = frame.shape[:2]
    print(f"Image size: {img_width} x {img_height}")

    # 3. Preprocess (Scale image to 160x160 for the model)
    # YOLOv8 expects pixel values 0-1, so we multiply by 1/255.0
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
    net.setInput(blob)

    # 4. Inference (The Magic Step)
    # Returns a raw matrix of shape (1, 5, 2100) or similar
    outputs = net.forward()
    print(f"Raw Output Shape: {outputs.shape}")
    max_conf = np.max(outputs[:, 4:, :]) 
    print(f"Highest confidence found in image: {max_conf:.4f}")
    # 5. Post-Processing (Decoding the Raw Matrix)
    # Transpose to shape (Rows, Cols) -> e.g., (2100, 5)
    outputs = np.transpose(np.squeeze(outputs))

    boxes = []
    confidences = []
    class_ids = []

    # Calculate scaling factors to map 160x160 detections back to 1280x720
    x_factor = img_width / INPUT_SIZE[0]
    y_factor = img_height / INPUT_SIZE[1]

    rows = outputs.shape[0]

    for i in range(rows):
        # The first 4 values are box coordinates. The rest are class probabilities.
        classes_scores = outputs[i][4:] 
        max_score = np.amax(classes_scores)

        if max_score >= CONF_THRESHOLD:
            class_id = np.argmax(classes_scores)
            
            # Extract box (Center_x, Center_y, Width, Height)
            x, y, w, h = outputs[i][0], outputs[i][1], outputs[i][2], outputs[i][3]

            # Convert to Top-Left Coordinate and Scale up
            left = int((x - w / 2) * x_factor)
            top = int((y - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)

            boxes.append([left, top, width, height])
            confidences.append(float(max_score))
            class_ids.append(class_id)

    # 6. Non-Maximum Suppression (Remove duplicate boxes for the same object)
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD)

    final_detections = []
    
    # Check if we have any detections
    if len(indices) > 0:
        for i in indices.flatten():
            box = boxes[i]
            (left, top, width, height) = box
            
            # Calculate x_max and y_max
            x_min, y_min = left, top
            x_max, y_max = left + width, top + height

            # Format for your main.py
            det = {
                "xyxy": [float(x_min), float(y_min), float(x_max), float(y_max)],
                "conf": confidences[i],
                "cls": int(class_ids[i]),
                "img_width": img_width,
                "img_height": img_height
            }
            final_detections.append(det)

            # Draw on image (Visualization)
            color = (0, 255, 0) # Green
            cv2.rectangle(frame, (left, top), (left + width, top + height), color, 2)
            
            label_name = CLASS_NAMES[class_ids[i]] if class_ids[i] < len(CLASS_NAMES) else str(class_ids[i])
            label = f"{label_name}: {confidences[i]:.2f}"
            cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # 7. Save Result
    cv2.imwrite("last_detection.jpg", frame)
    print(f"Saved visualization to last_detection.jpg. Detected: {len(final_detections)} objects.")

    # 8. Show Result Window (Optional - works on Desktop, might fail on Headless Pi)
    try:
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1) # Important!
    except Exception:
        pass # Ignore if no GUI available

    return final_detections

if __name__ == "__main__":
    try:
        dets = run_model_on_one_frame()
        print("Detections returned:", dets)
    except Exception as e:
        print(f"Error: {e}")