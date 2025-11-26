import waste_position
import cv2
from qr import _read_image, detect_and_decode_qr
import ultrasonic_sensor



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
    ultrasonic_data = ultrasonic_sensor.get_distance()
    print("Ultrasonic Sensor Data:", ultrasonic_data)

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

print(qr_contents)





dets = waste_position.run_model_on_one_frame()
for d in dets:
    print(d)
    


                    
