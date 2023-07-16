# Tests require running mineros and minecraft
import math
import threading
import time
import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State

from geometry_msgs.msg import PoseStamped, PoseArray, Pose


class MovementTestNode(Node):
    def __init__(self):
        super().__init__('movement_tester')
        self.position: PoseStamped = None

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/mineros/local_position/pose',
            self.position_cb,
            10
        )

        self.set_position_xyz_pub = self.create_publisher(
            PoseStamped,
            '/mineros/set_position/xyz',
            10
        )

        self.set_position_xz_pub = self.create_publisher(
            PoseStamped,
            '/mineros/set_position/xz',
            10
        )

        self.set_position_composite_xz_pub = self.create_publisher(
            PoseArray,
            '/mineros/set_position/composite/xz',
            10
        )

        threading.Thread(target=self.tests).start()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def set_position_xyz(self, x: float, y: float, z: float):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        self.set_position_xyz_pub.publish(pose)

    def set_position_xz(self, x: float, z: float):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.z = z

        self.set_position_xz_pub.publish(pose)

    def tests(self):
        for i in range(5):
            self.test_position_xyz(1, True)
            self.test_position_xz(1, True)
        self.get_logger().info('Stress test passed')

        self.test_position_xyz(5)
        self.test_position_xz(5)

        self.destroy_node()
        rclpy.shutdown()

    def test_position_xyz(self, sleep, stress_test=False):
        while self.position is None:
            time.sleep(0.1)

        ps = PoseStamped()
        ps.pose.position.x = self.position.pose.position.x + 1
        ps.pose.position.y = self.position.pose.position.y - 1
        ps.pose.position.z = self.position.pose.position.z + 1
        self.set_position_xyz_pub.publish(ps)

        time.sleep(sleep)
        if not stress_test:
            assert self.distance_between_2_points(self.position, ps) <= 1
        self.get_logger().info("test_position_xyz passed")

    def test_position_xz(self, sleep, stress_test=False):
        while self.position is None:
            time.sleep(0.1)

        ps = PoseStamped()
        ps.pose.position.x = self.position.pose.position.x + 1
        ps.pose.position.y = self.position.pose.position.y
        ps.pose.position.z = self.position.pose.position.z + 1
        self.set_position_xyz_pub.publish(ps)

        time.sleep(sleep)
        if not stress_test:
            assert self.distance_between_2_points(self.position, ps) <= 1

        self.get_logger().info("test_position_xz passed")


    def distance_between_2_points(self, point1: PoseStamped, point2: PoseStamped):
        return math.sqrt(
            (point1.pose.position.x - point2.pose.position.x)**2 +
            (point1.pose.position.y - point2.pose.position.y)**2 +
            (point1.pose.position.z - point2.pose.position.z)**2
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovementTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
