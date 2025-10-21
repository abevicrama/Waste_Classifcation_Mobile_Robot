
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from wastebot.utils import load_config, clamp, deg2us

try:
    import busio
    from board import SCL, SDA
    from adafruit_pca9685 import PCA9685
    HAVE_PCA = True
except Exception:
    HAVE_PCA = False

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.declare_parameter('config', 'config/bins.yaml')
        self.cfg = load_config(self.get_parameter('config').get_parameter_value().string_value)
        self.us_min = self.cfg['arm']['microseconds']['min']
        self.us_max = self.cfg['arm']['microseconds']['max']

        if HAVE_PCA:
            i2c_bus_id = self.cfg['arm'].get('i2c_bus', 1)
            i2c = busio.I2C(SCL, SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = self.cfg['arm']['frequency_hz']
        else:
            self.pca = None
            self.get_logger().warn('PCA9685 not available on this system; running in dry mode.')

        ch = self.cfg['arm']['channels']
        self.ch_base = ch['base']; self.ch_shoulder = ch['shoulder']
        self.ch_elbow = ch['elbow']; self.ch_wrist = ch['wrist']; self.ch_gripper = ch['gripper']

        # Helpers/services
        self.create_service(Trigger, 'arm_up', self._srv_up)
        self.create_service(Trigger, 'arm_down', self._srv_down)
        self.create_service(Trigger, 'arm_forward', self._srv_forward)
        self.create_service(Trigger, 'arm_backward', self._srv_backward)
        self.create_service(Trigger, 'arm_catch', self._srv_catch)
        self.create_service(Trigger, 'arm_release', self._srv_release)

        # Initialize to STOW
        self.named_pose('STOW')

    # ------------- Low-level --------------
    def _write_deg(self, ch, deg):
        us = deg2us(deg, self.us_min, self.us_max)
        duty = int(us * 4096 * self.cfg['arm']['frequency_hz'] / 1_000_000)
        if self.pca:
            self.pca.channels[ch].duty_cycle = duty
        self.get_logger().debug(f'ch{ch} -> {deg}° ({us}us)')

    def set_angles(self, base=None, shoulder=None, elbow=None, wrist=None, gripper=None, delay=0.3):
        if base is not None:    self._write_deg(self.ch_base, clamp(base, 0, 180))
        if shoulder is not None:self._write_deg(self.ch_shoulder, clamp(shoulder, 0, 180))
        if elbow is not None:   self._write_deg(self.ch_elbow, clamp(elbow, 0, 180))
        if wrist is not None:   self._write_deg(self.ch_wrist, clamp(wrist, 0, 180))
        if gripper is not None: self._write_deg(self.ch_gripper, clamp(gripper, 0, 180))
        time.sleep(delay)

    def named_pose(self, name: str):
        pose = self.cfg['arm']['named_poses'][name]
        self.set_angles(**pose)

    # ------------- Semantics --------------
    def up(self):
        self.set_angles(shoulder=70, elbow=90)

    def down(self):
        self.set_angles(shoulder=120, elbow=130)

    def forward(self):
        self.set_angles(elbow=100, wrist=90)

    def backward(self):
        self.set_angles(elbow=80, wrist=110)

    def catch(self):
        # close gripper
        self.set_angles(gripper=25)

    def release(self):
        # open gripper
        self.set_angles(gripper=70)

    # ------------- ROS srvs ----------------
    def _srv_up(self, req, res): self.up(); res.success=True; res.message='up'; return res
    def _srv_down(self, req, res): self.down(); res.success=True; res.message='down'; return res
    def _srv_forward(self, req, res): self.forward(); res.success=True; res.message='forward'; return res
    def _srv_backward(self, req, res): self.backward(); res.success=True; res.message='backward'; return res
    def _srv_catch(self, req, res): self.catch(); res.success=True; res.message='catch'; return res
    def _srv_release(self, req, res): self.release(); res.success=True; res.message='release'; return res

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
