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

        self.set_position_pub = self.create_publisher(
            PoseStamped,
            '/mineros/set_position',
            10
        )

        self.set_position_composite_pub = self.create_publisher(
            PoseArray,
            '/mineros/set_position/composite',
            10
        )

        threading.Thread(target=self.tests).start()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        for i in range(5):
            self.test_position_xyz(1, True)
            self.test_position_xz(1, True)
        self.get_logger().info('Stress test passed')

        self.test_position_xyz(5)
        self.test_position_xz(5)
        self.test_composite()

        self.destroy_node()
        rclpy.shutdown()

    def test_position_xyz(self, sleep, stress_test=False):
        while self.position is None:
            time.sleep(0.1)

        ps = PoseStamped()
        ps.pose.position.x = self.position.pose.position.x + 1
        ps.pose.position.y = self.position.pose.position.y - 1
        ps.pose.position.z = self.position.pose.position.z + 1
        self.set_position_pub.publish(ps)

        time.sleep(sleep)
        if not stress_test:
            assert self.distance_between_2_points(self.position, ps) <= 1
        self.get_logger().info("test_position_xyz passed")

    def test_position_xz(self, sleep, stress_test=False):
        while self.position is None:
            time.sleep(0.1)

        ps = PoseStamped()
        ps.pose.position.x = self.position.pose.position.x + 1
        ps.pose.position.y = -1.0
        ps.pose.position.z = self.position.pose.position.z + 1
        self.set_position_pub.publish(ps)

        time.sleep(sleep)
        if not stress_test:
            ps.pose.position.y = self.position.pose.position.y
            assert self.distance_between_2_points(self.position, ps) <= 1

        self.get_logger().info("test_position_xz passed")

    def test_composite(self):
        poses = []

        for i in range(10):
            p = Pose()
            p.position.x = self.position.pose.position.x + i
            p.position.y = -1.0
            p.position.z = self.position.pose.position.z + i
            poses.append(p)
        
        pa = PoseArray()
        pa.poses = poses
        
        self.set_position_composite_pub.publish(pa)
        
        time.sleep(10)
        final_ps = PoseStamped()
        final_ps.pose = poses[-1]
        final_ps.pose.position.y = self.position.pose.position.y
        
        assert self.distance_between_2_points(self.position, final_ps)
        
        self.get_logger().info('composite_test passed')

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
