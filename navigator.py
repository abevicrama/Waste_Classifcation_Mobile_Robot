
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from wastebot.utils import load_config, yaw_to_quat

class Navigator(Node):
    """
    Thin wrapper around Nav2 Simple Commander to provide:
      - navigate_to(x, y, yaw)
      - navigate_to_bin(color) using config bin_locations
    Requires Nav2 to be running and SLAM/localization to be active.
    """
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('config', 'config/bins.yaml')
        self.cfg = load_config(self.get_parameter('config').get_parameter_value().string_value)
        self.nav = BasicNavigator()
        self.frame_id = 'map'

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        q = yaw_to_quat(float(yaw))
        p.pose.orientation.z = q['z']
        p.pose.orientation.w = q['w']
        return p

    def navigate_to(self, x, y, yaw=0.0, timeout_sec=120.0):
        goal = self._pose(x, y, yaw)
        self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = self.nav.getResult()
        return result

    def navigate_to_bin(self, color: str, timeout_sec=120.0):
        bins = self.cfg.get('bin_locations', {})
        if color not in bins:
            raise ValueError(f'bin color {color} not in config')
        pose = bins[color]
        result = self.navigate_to(pose['x'], pose['y'], pose['yaw'], timeout_sec=timeout_sec)
        return result

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
