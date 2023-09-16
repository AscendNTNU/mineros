import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose

from mineros_interfaces.srv import FindBlocks, FurnaceInfo
from mineros_interfaces.msg import Furnace, Item


class InteractionTestNode(Node):
    def __init__(self):
        super().__init__('interaction_tester')

        self.position: PoseStamped = None

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/mineros/local_position/pose',
            self.position_cb,
            10
        )

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )

        self.furnace_info_client = self.create_client(
            FurnaceInfo,
            '/mineros/interaction/furnace_info'
        )

        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        self.test_furnace_info()
        
        self.destroy_node()
        rclpy.shutdown()

    def test_furnace_info(self):
        self.get_logger().info('Testing furnace info')
        FURNACE_LOC = (754.452, 102.0, 1666.395)
        pose: pose = Pose()
        pose.position.x = FURNACE_LOC[0]
        pose.position.y = FURNACE_LOC[1]
        pose.position.z = FURNACE_LOC[2]

        req = FurnaceInfo.Request()
        req.block_pose = pose
        future = self.furnace_info_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        furace: Furnace = future.result()
        self.get_logger().info(f'{furace}')
        assert furace.success

def main(args=None):
    rclpy.init(args=args)
    node = InteractionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()