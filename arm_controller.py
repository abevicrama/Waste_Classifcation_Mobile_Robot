"""Arm controller module with a reusable API for main.py.

This refactors the previous keyboard-driven script into a module exposing
functions/classes that main.py can import and use to rotate the base
by a percentage, move joints, and control the gripper.
"""

import time
from typing import Optional

try:
    from gpiozero import Servo
    from gpiozero.pins.pigpio import PiGPIOFactory
    factory = PiGPIOFactory()
    IS_PIGPIO_READY = True
except Exception:
    IS_PIGPIO_READY = False


# GPIO BCM pin numbers
PIN_STAGE_ROTATION = 5      # Base/Stage Rotation (Yaw)
PIN_ARM_FORWARD_BACK = 6    # Shoulder Pitch
PIN_ARM_UP_DOWN = 26        # Elbow Pitch
PIN_FINGER_GRIP = 19        # Gripper/Claw


class ArmController:
    """High-level arm controller suitable for programmatic control."""

    def __init__(self) -> None:
        if not IS_PIGPIO_READY:
            raise RuntimeError("pigpio not ready. Ensure 'sudo pigpiod' is running on the Pi.")

        # Servo setup with custom pulse widths (as in original script)
        self.base_servo = Servo(PIN_STAGE_ROTATION, pin_factory=factory,
                                min_pulse_width=0.50/1000, max_pulse_width=1.39/1000)
        self.shoulder_servo = Servo(PIN_ARM_FORWARD_BACK, pin_factory=factory,
                                    min_pulse_width=1.17/1000, max_pulse_width=2.28/1000)
        self.elbow_servo = Servo(PIN_ARM_UP_DOWN, pin_factory=factory,
                                 min_pulse_width=1.17/1000, max_pulse_width=2.28/1000)
        self.gripper_servo = Servo(PIN_FINGER_GRIP, pin_factory=factory,
                                   min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

    @staticmethod
    def _clamp(val: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, val))

    def initialize(self) -> None:
        """Move servos to safe initial positions: base~30°, shoulder/elbow~110°, gripper~50°."""
        self.base_servo.value = 0.0       # center of limited range (~30°)
        self.shoulder_servo.value = 0.0   # ~110°
        self.elbow_servo.value = 0.0      # ~110°
        self.gripper_servo.value = -0.44  # ~50° (open)
        time.sleep(0.5)

    def set_gripper(self, state: str) -> None:
        """Set gripper: 'catch' -> closed, 'release' -> open."""
        if state == 'catch':
            self.gripper_servo.value = 1.0
        elif state == 'release':
            self.gripper_servo.value = -1.0

    def rotate_stage_percent(self, percent: float) -> float:
        """Rotate base by percentage relative to the initial ~30° position.

        Mapping: percent in [0,100] -> servo.value in [-1.0, 1.0]
        - 0%   => left-most (~0°)
        - 50%  => center (~30° baseline)
        - 100% => right-most (~80°)

        Returns the applied `servo.value` for logging.
        """
        p = max(0.0, min(100.0, float(percent)))
        servo_value = (p / 50.0) - 1.0  # 0%->-1.0, 50%->0.0, 100%->1.0
        servo_value = self._clamp(servo_value)
        self.base_servo.value = servo_value
        return servo_value

    def cleanup(self) -> None:
        """Detach servos to de-energize."""
        try:
            self.base_servo.detach()
            self.shoulder_servo.detach()
            self.elbow_servo.detach()
            self.gripper_servo.detach()
        except Exception:
            pass


# Convenience module-level API for simple use from main.py
_controller: Optional[ArmController] = None

def setup() -> None:
    """Create and initialize the arm controller (idempotent)."""
    global _controller
    if _controller is None:
        _controller = ArmController()
        _controller.initialize()

def rotate_stage_percent(percent: float) -> float:
    """Rotate the base to the given percentage using the shared controller."""
    if _controller is None:
        setup()
    return _controller.rotate_stage_percent(percent)

def set_gripper(state: str) -> None:
    if _controller is None:
        setup()
    _controller.set_gripper(state)

def cleanup() -> None:
    global _controller
    if _controller is not None:
        _controller.cleanup()
        _controller = None


# Optional: retain keyboard control when running this file directly
if __name__ == '__main__':
    if not IS_PIGPIO_READY:
        print("pigpio not ready. Run 'sudo pigpiod' and re-execute.")
        raise SystemExit(1)

    setup()
    print("Arm initialized. Rotate base to 75% as a quick demo.")
    applied = rotate_stage_percent(75.0)
    print(f"Applied base servo value: {applied:.2f}")
    time.sleep(1.0)
    cleanup()
