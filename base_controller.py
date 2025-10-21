
import time
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from wastebot.utils import load_config, clamp

try:
    import RPi.GPIO as GPIO
    HAVE_GPIO = True
except Exception:
    HAVE_GPIO = False

class BaseController(Node):
    """
    Base controller for 4WD differential drive.
    Modes:
      - GPIO mode: directly toggles motor driver pins (TB6612/L298N).
      - /cmd_vel bridge: publish Twist if you already have a hardware driver.
    """
    def __init__(self):
        super().__init__('base_controller')
        self.declare_parameter('config', 'config/bins.yaml')
        self.declare_parameter('use_cmd_vel_bridge', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.cfg = load_config(self.get_parameter('config').get_parameter_value().string_value)

        self.use_gpio = self.cfg.get('gpio', {}).get('enable_gpio_mode', True) and HAVE_GPIO and not self.get_parameter('use_cmd_vel_bridge').value
        self.turn_rate = self.cfg.get('base_calibration', {}).get('turn_rate_deg_per_sec', 120.0)
        self.straight_scale = self.cfg.get('base_calibration', {}).get('straight_scale_mps', 0.25)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.lock = threading.Lock()

        if self.use_gpio:
            self.get_logger().info('Using GPIO motor control mode')
            self._gpio_setup()
        else:
            self.get_logger().info('Using /cmd_vel bridge mode (no direct GPIO)')

        # simple services
        self.srv_stop = self.create_service(Trigger, 'stop', self._srv_stop)

    # ---------------- GPIO ----------------
    def _gpio_setup(self):
        g = self.cfg['gpio']
        GPIO.setmode(GPIO.BCM)
        self.AIN1, self.AIN2, self.PWMA = g['left']['in1'], g['left']['in2'], g['left']['pwm']
        self.BIN1, self.BIN2, self.PWMB = g['right']['in1'], g['right']['in2'], g['right']['pwm']
        for p in [self.AIN1, self.AIN2, self.PWMA, self.BIN1, self.BIN2, self.PWMB]:
            GPIO.setup(p, GPIO.OUT)
        self.pwmA = GPIO.PWM(self.PWMA, g.get('pwm_freq_hz', 20000)); self.pwmA.start(0)
        self.pwmB = GPIO.PWM(self.PWMB, g.get('pwm_freq_hz', 20000)); self.pwmB.start(0)

    def _gpio_left(self, speed):
        # speed in [-1..1]
        fwd = speed >= 0
        GPIO.output(self.AIN1, fwd)
        GPIO.output(self.AIN2, not fwd)
        self.pwmA.ChangeDutyCycle(min(abs(speed)*100, 100))

    def _gpio_right(self, speed):
        fwd = speed >= 0
        GPIO.output(self.BIN1, fwd)
        GPIO.output(self.BIN2, not fwd)
        self.pwmB.ChangeDutyCycle(min(abs(speed)*100, 100))

    # ------------- Bridge publish ---------
    def _pub_cmd(self, lin_x, ang_z):
        msg = Twist()
        msg.linear.x = float(clamp(lin_x, -1.0, 1.0))
        msg.angular.z = float(clamp(ang_z, -2.0, 2.0))
        self.cmd_pub.publish(msg)

    # ------------- Public API -------------
    def go_straight(self, speed=0.5, seconds=None, distance=None):
        speed = clamp(speed, -1.0, 1.0)
        if seconds is None and distance is not None:
            seconds = float(distance) / max(1e-3, (self.straight_scale * abs(speed)))
        elif seconds is None:
            seconds = 1.0
        self._drive(speed, speed, seconds)

    def go_backward(self, speed=0.5, seconds=1.0):
        self._drive(-abs(speed), -abs(speed), seconds)

    def turn_left(self, angle_deg, speed=0.5):
        rate = self.turn_rate * abs(speed)  # deg/sec
        seconds = abs(angle_deg) / max(1e-3, rate)
        self._drive(-abs(speed), abs(speed), seconds)

    def turn_right(self, angle_deg, speed=0.5):
        rate = self.turn_rate * abs(speed)
        seconds = abs(angle_deg) / max(1e-3, rate)
        self._drive(abs(speed), -abs(speed), seconds)

    def stop(self):
        if self.use_gpio:
            self._gpio_left(0); self._gpio_right(0)
        # also publish zero to cmd_vel
        self._pub_cmd(0.0, 0.0)

    # ------------- Core drive -------------
    def _drive(self, left_speed, right_speed, seconds):
        with self.lock:
            t0 = time.time()
            while (time.time() - t0) < seconds:
                if self.use_gpio:
                    self._gpio_left(left_speed)
                    self._gpio_right(right_speed)
                else:
                    # convert to Twist: lin = avg, ang = diff / wheel_base_proxy
                    lin = (left_speed + right_speed) * 0.5
                    ang = (right_speed - left_speed) * 2.0
                    self._pub_cmd(lin, ang)
                time.sleep(0.02)
            self.stop()

    # ------------- ROS srv ----------------
    def _srv_stop(self, request, response):
        self.stop()
        response.success = True
        response.message = "Stopped."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
