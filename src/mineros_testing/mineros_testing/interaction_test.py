import rclpy
from rclpy.node import Node
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped, Pose

from mineros_inter.srv import FindBlocks, FurnaceInfo, FurnaceUpdate, Recipe
from mineros_inter.msg import Furnace, Item

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
        
        self.get_recipe_client = self.create_client(
            Recipe,
            '/mineros/interaction/recipe',
        )
        
        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )
        
        self.furnace_loc = (754.452, 102.0, 1666.395)
        
        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        # self.test_furnace_info()
        # self.test_update_furnace()
        self.test_get_recipe()
        
        self.destroy_node()
        rclpy.shutdown()
    
    def test_get_recipe(self):
        req = Recipe.Request()
        
        diamond_pickaxe = Item()
        diamond_pickaxe.id = 799
        diamond_pickaxe.count = 1
        req.item = diamond_pickaxe
        
        crafting_table_loc = self.find_blocks(182, 1)[0]
        assert crafting_table_loc is not None
        req.crafting_table_location = crafting_table_loc
        req.crafting_table = True
        
        future = self.get_recipe_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'{future.result()}')
        

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
        
    def find_blocks(self, blockid: int, count: int) -> List[Pose]:
        self.get_logger().info('Find blocks')
        block_search = FindBlocks.Request()
        block_search.blockid = blockid
        block_search.count = count

        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()
        return blocks.blocks.poses

def main(args=None):
    rclpy.init(args=args)
    node = InteractionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()