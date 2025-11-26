import waste_position
import cv2
from qr import _read_image, detect_and_decode_qr
# import ultrasonic_sensor
import arm_controller as arm



waste_hange = False

def check_bin_by_qr(detections):
    for det in detections:
        deta = det.get("data", "")
        print(f"Detected QR Code Data: {deta}")
        waste_put_in_bin(deta)
        
        
def waste_put_in_bin(deta):
    if waste_hange == True:
        if deta == "waste 1":
        # put the waste handling code here
            print("Handling waste 1")
        elif deta == "waste 2":
        # put the waste handling code here
            print("Handling waste 2")
        elif deta == "waste 3":
        # put the waste handling code here
            print("Handling waste 3")
        elif deta == "waste 4":
            # put the waste handling code here
            print("Handling waste 4")
        else:
            print("No specific waste handling code for this QR code.")
    else:
        #go back to normal operation to search for waste
        print("Resuming normal operation.")


try:
    #get ultrasonic sensor data
    #ultrasonic_data = ultrasonic_sensor.get_distance()
    #print("Ultrasonic Sensor Data:", ultrasonic_data)
    ultrasonic_data = 12
      # Example flag for waste handling
    
    
    if ultrasonic_data < 20:  # example threshold in cm
        print("Object too close! Stopping robot.")
        # Code to stop the robot would go here
        
        try:
            # get camera 

            # Read the image
            qr_box = cv2.imread("Untitled.png")
            detections = detect_and_decode_qr(qr_box)
            if detections:
                check_bin_by_qr(detections)
            else:
                print("No QR codes detected.")
                
        except Exception as e:
            print(f"Error reading image or detecting QR code: {e}")
    
except ImportError:
    print("Ultrasonic sensor module not found.")

   # returns a BGR numpy array
qr_contents = _read_image("Untitled.png")

#print(qr_contents)





"""Run YOLO on one frame, compute center percentage, rotate arm stage."""
dets = waste_position.run_model_on_one_frame()
det = dets[0] if dets else None

def calculate_center_xyxy(xyxy):
    x_min, y_min, x_max, y_max = xyxy
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    return center_x, center_y    
    
def persentage_position(det):
    img_width = det.get("img_width")
    img_height = det.get("img_height")
    xyxy = det.get("xyxy")
    center_x, center_y = calculate_center_xyxy(xyxy)
    perc_x = (center_x / img_width) * 100
    perc_y = (center_y / img_height) * 100
    return perc_x, perc_y

p_x, p_y = persentage_position(det) if det else (50.0, 50.0)
print(f"Object center (%): x={p_x:.2f}%, y={p_y:.2f}%")

# Rotate base stage to p_x percentage relative to baseline (~30°)
try:
    arm.setup()
    applied_val = arm.rotate_stage_percent(p_x)
    print(f"Applied base servo value: {applied_val:.2f}")
except Exception as e:
    print(f"Arm control unavailable: {e}")
finally:
    try:
        arm.cleanup()
    except Exception:
        pass
