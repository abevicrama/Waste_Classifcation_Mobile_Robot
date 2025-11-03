import time
import readchar
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# ====================================================================
# 1. Configuration and Setup
# ====================================================================

# Servo motor control uses the pigpio library for high-accuracy PWM.
# NOTE: The pigpio daemon MUST be running on your Pi (run 'sudo pigpiod')
try:
    factory = PiGPIOFactory()
    IS_PIGPIO_READY = True
except OSError:
    print("\n\n#####################################################")
    print("FATAL ERROR: Could not connect to the pigpio daemon.")
    print("Please ensure you have run 'sudo pigpiod' in your terminal.")
    print("#####################################################\n")
    IS_PIGPIO_READY = False
    # If the daemon isn't ready, the script will exit in the main block.


# Define the GPIO BCM pin numbers for each of the four servos.
PIN_STAGE_ROTATION = 5      # 1. Base/Stage Rotation (Yaw)
PIN_ARM_FORWARD_BACK = 6    # 2. Arm Forward/Backward (Shoulder Pitch)
PIN_ARM_UP_DOWN = 26        # 3. Arm Up/Down (Elbow Pitch)
PIN_FINGER_GRIP = 19        # 4. Finger/Gripper (Claw)

# --- Pulse Width Setup (Limited Angular Range) ---
# We use custom pulse widths to map the limited angular range (+/- 50 deg from initial)
# to the Servo control values of -1.0 to 1.0. 
if IS_PIGPIO_READY:
    # 1. Base Rotation (Initial: 30 deg. Range: ~0 deg to 80 deg)
    base_servo = Servo(PIN_STAGE_ROTATION, pin_factory=factory, 
                       min_pulse_width=0.50/1000, max_pulse_width=1.39/1000)

    # 2 & 3. Arm Movement (Initial: 110 deg. Range: ~60 deg to 160 deg)
    shoulder_servo = Servo(PIN_ARM_FORWARD_BACK, pin_factory=factory,
                           min_pulse_width=1.17/1000, max_pulse_width=2.28/1000)

    elbow_servo = Servo(PIN_ARM_UP_DOWN, pin_factory=factory,
                        min_pulse_width=1.17/1000, max_pulse_width=2.28/1000)

    # 4. Gripper/Finger (Initial: 50 deg. Full range for catch/release)
    gripper_servo = Servo(PIN_FINGER_GRIP, pin_factory=factory,
                          min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)


# ====================================================================
# 2. Control Functions (Mapped to servo angle values: -1.0 to 1.0)
# ====================================================================

# Global step size for continuous movement
STEP_SIZE = 0.05 

# Helper function for setting servo value safely
def set_servo_value(servo, value, name):
    """Sets the servo value and clamps it between -1.0 and 1.0."""
    value = max(-1.0, min(1.0, value))
    servo.value = value
    # print(f"[{name:15}] Set to value: {value:.2f}") # Commented out for cleaner console during control
    return value

def initialize_arm():
    """Moves all servos to the safe initial positions: 30, 110, 110, 50."""
    print("\n--- Initializing Arm to Start Position (30, 110, 110, 50) ---")
    
    # Base/Shoulder/Elbow are set to 0.0, which is the center of their limited range (the initial angle)
    base_value = set_servo_value(base_servo, 0.0, "Base Rotation")
    shoulder_value = set_servo_value(shoulder_servo, 0.0, "Arm Fwd/Bwd")
    elbow_value = set_servo_value(elbow_servo, 0.0, "Arm Up/Down")
    
    # Gripper needs to be set to the value corresponding to 50 deg (-0.44)
    gripper_value = set_servo_value(gripper_servo, -0.44, "Gripper")
    
    time.sleep(1.0)
    print("Initialization Complete.")
    return base_value, shoulder_value, elbow_value, gripper_value

def set_gripper(state):
    """Controls the gripper discretely (catch=1.0, release=-1.0)."""
    if state == 'catch':
        gripper_value = set_servo_value(gripper_servo, 1.0, "Gripper")
        print("COMMAND: Gripper CATCH")
    elif state == 'release':
        gripper_value = set_servo_value(gripper_servo, -1.0, "Gripper")
        print("COMMAND: Gripper RELEASE")
    return gripper_value

# ====================================================================
# 3. Main Keyboard Control Loop
# ====================================================================

if __name__ == '__main__':
    if not IS_PIGPIO_READY:
        print("Cannot run control loop without pigpio daemon. Exiting.")
        exit()

    # Initialize current servo values to start position
    b_val, s_val, e_val, g_val = initialize_arm()
    
    print("\n🤖 Robot Arm Keyboard Control")
    print("-" * 35)
    print(" W (Elbow Up)   |   Q (Shoulder FWD) ")
    print(" S (Elbow Down) |   E (Shoulder BWD) ")
    print(" A (Base Left)  |   D (Base Right)   ")
    print(" Z (Release)    |   C (Catch)        ")
    print(" R (Reset)      |   X (Quit)         ")
    print("-" * 35)
    
    current_key = ''

    try:
        while current_key != 'x':
            # Read a single key press without waiting for Enter
            current_key = readchar.readkey().lower()

            if current_key == 'w':
                e_val += STEP_SIZE
                e_val = set_servo_value(elbow_servo, e_val, "Arm Up/Down")
                print(f"COMMAND: Elbow UP -> {e_val:.2f}")

            elif current_key == 's':
                e_val -= STEP_SIZE
                e_val = set_servo_value(elbow_servo, e_val, "Arm Up/Down")
                print(f"COMMAND: Elbow DOWN -> {e_val:.2f}")

            elif current_key == 'q':
                s_val += STEP_SIZE
                s_val = set_servo_value(shoulder_servo, s_val, "Arm Fwd/Bwd")
                print(f"COMMAND: Shoulder FWD -> {s_val:.2f}")

            elif current_key == 'e':
                s_val -= STEP_SIZE
                s_val = set_servo_value(shoulder_servo, s_val, "Arm Fwd/Bwd")
                print(f"COMMAND: Shoulder BWD -> {s_val:.2f}")

            elif current_key == 'a':
                b_val -= STEP_SIZE
                b_val = set_servo_value(base_servo, b_val, "Base Rotation")
                print(f"COMMAND: Base LEFT -> {b_val:.2f}")

            elif current_key == 'd':
                b_val += STEP_SIZE
                b_val = set_servo_value(base_servo, b_val, "Base Rotation")
                print(f"COMMAND: Base RIGHT -> {b_val:.2f}")

            elif current_key == 'z':
                g_val = set_gripper('release')

            elif current_key == 'c':
                g_val = set_gripper('catch')
            
            elif current_key == 'r':
                b_val, s_val, e_val, g_val = initialize_arm()
                print("COMMAND: RESET")

            time.sleep(0.01) # Small delay for stability

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        
    finally:
        print("\n--- Cleanup: Stopping Servo Signals ---")
        # De-energize the servos by setting value to None.
        if IS_PIGPIO_READY:
            print("Servos detached.")
            base_servo.detach()
            shoulder_servo.detach()
            elbow_servo.detach()
            gripper_servo.detach()
        else:
            print("Skipped servo detachment due to pigpio connection failure.")
