import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose

from mineros_interfaces.srv import FindBlocks, FurnaceInfo, FurnaceUpdate
from mineros_interfaces.msg import Furnace, Item

from .utils import item_equal


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
        
        self.furnace_update_client = self.create_client(
            FurnaceUpdate,
            '/mineros/interaction/furnace_update',
        )
        
        self.furnace_loc = (754.452, 102.0, 1666.395)
        
        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        self.test_furnace_info()
        self.test_update_furnace()
        
        self.destroy_node()
        rclpy.shutdown()

    def test_furnace_info(self):
        self.get_logger().info('Testing furnace info')
        pose: pose = Pose()
        pose.position.x = self.furnace_loc[0]
        pose.position.y = self.furnace_loc[1]
        pose.position.z = self.furnace_loc[2]

        req = FurnaceInfo.Request()
        req.block_pose = pose
        future = self.furnace_info_client.call_async(req)
        self.get_logger().info('Waiting for furnace info')
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Got furnace info')
        furace: Furnace = future.result()
        self.get_logger().info(f'{furace}')
        assert furace.success
        return furace.furnace
        
    def test_update_furnace(self):
        req = FurnaceUpdate.Request()
        pose: pose = Pose()
        pose.position.x = self.furnace_loc[0]
        pose.position.y = self.furnace_loc[1]
        pose.position.z = self.furnace_loc[2]
        
        req.block_pose = pose
        req.furnace = Furnace()
        req.furnace.fuel_item = Item()
        req.furnace.fuel_item.id = 110
        req.furnace.fuel_item.count = 2
        req.furnace.input_item = Item()
        req.furnace.input_item.id = 110
        req.furnace.input_item.count = 2
        req.furnace.output_item = Item()
        req.furnace.output_item.id = 0
        req.furnace.output_item.count = 0
        
        future = self.furnace_update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success
        new: Furnace = self.test_furnace_info()
        assert item_equal(new.fuel_item, req.furnace.fuel_item)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = InteractionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()