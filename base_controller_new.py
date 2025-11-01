import time
import readchar
from gpiozero import Motor

# ====================================================================
# Motor Configuration (MUST match your hardware setup)
# ====================================================================
# LEFT MOTOR PINS
ENA_PIN = 12       
LEFT_IN1_PIN = 17  
LEFT_IN2_PIN = 18  

# RIGHT MOTOR PINS
ENB_PIN = 13       
RIGHT_IN3_PIN = 27 
RIGHT_IN4_PIN = 22 

# Initialize the individual Motor objects
left_motor = Motor(forward=LEFT_IN1_PIN, backward=LEFT_IN2_PIN, enable=ENA_PIN)
right_motor = Motor(forward=RIGHT_IN3_PIN, backward=RIGHT_IN4_PIN, enable=ENB_PIN)

# ====================================================================
# Movement Parameters
# ====================================================================
DEFAULT_SPEED = 0.6  # Base speed for movement (0.0 to 1.0)
TURN_POWER = 0.5     # Base power for turning (spinning)
TURN_BIAS = 1.0      # Set to 1.0 for turning in place (skid steer)

# Define Arrow Key Constants (Specific to readchar)
KEY_UP    = readchar.key.UP
KEY_DOWN  = readchar.key.DOWN
KEY_LEFT  = readchar.key.LEFT
KEY_RIGHT = readchar.key.RIGHT
KEY_STOP  = ' '      # Spacebar for stopping
KEY_QUIT  = 'q'      # Q key to exit the program

# ====================================================================
# Movement Functions (Reused from previous code)
# ====================================================================

def move_forward(speed):
    """Moves the robot straight forward."""
    left_motor.forward(speed)
    right_motor.backward(speed)

def move_backward(speed):
    """Moves the robot straight backward."""
    left_motor.backward(speed)
    right_motor.forward(speed)

def turn_right(power, turn_bias):
    """Turns the robot right in place (Left FWD, Right BWD)."""
    left_motor.forward(power)
    right_motor.forward(power * turn_bias)

def turn_left(power, turn_bias):
    """Turns the robot left in place (Left BWD, Right FWD)."""
    left_motor.backward(power * turn_bias)
    right_motor.backward(power)

def stop():
    """Immediately stops the robot."""
    left_motor.stop()
    right_motor.stop()

# ====================================================================
# Main Control Loop
# ====================================================================

if __name__ == '__main__':
    print("🤖 Robot Keyboard Control")
    print("-" * 30)
    print(f"UP: Forward | DOWN: Backward | LEFT: Turn Left | RIGHT: Turn Right")
    print(f"SPACE: Stop | Q: Quit")
    print("-" * 30)
    
    current_key = ''

    try:
        while current_key != KEY_QUIT:
            # Read a single key press without waiting for Enter
            current_key = readchar.readkey()
            
            # Reset speed to prevent continuous movement when key is released (for basic control)
            stop()

            if current_key == KEY_UP:
                move_forward(DEFAULT_SPEED)
                print(f"COMMAND: Forward ({DEFAULT_SPEED:.1f})")
                
            elif current_key == KEY_DOWN:
                move_backward(DEFAULT_SPEED)
                print(f"COMMAND: Backward ({DEFAULT_SPEED:.1f})")
                
            elif current_key == KEY_LEFT:
                turn_left(TURN_POWER, TURN_BIAS)
                print(f"COMMAND: Turn Left ({TURN_POWER:.1f})")
                
            elif current_key == KEY_RIGHT:
                turn_right(TURN_POWER, TURN_BIAS)
                print(f"COMMAND: Turn Right ({TURN_POWER:.1f})")

            elif current_key == KEY_STOP:
                stop()
                print("COMMAND: STOP")
            
            # Optional: Add a short delay to stabilize
            time.sleep(0.01)

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        
    finally:
        # Final cleanup and stop
        stop()
        print("\nExiting program. Motors stopped.")
