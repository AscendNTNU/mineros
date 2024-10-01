import time
import rclpy
from rclpy.node import Node

from typing import Any
from geometry_msgs.msg import PoseStamped
from mineros_inter.srv import (BlockInfo, BotPos, MoveTo, Craft, Inventory, Recipe, FindBlocks, MineBlock, PlaceBlock)

class MyFirstFSM(Node):
    """
    This class encompases the skeleton of a ROS2 node. It inherits from the class Node, which is the base class
    for all nodes in ROS2. The class Node is defined in the rclpy package. The class Node has a constructor that
    takes the name of the node as an argument. The name of the node is the name that will be used to identify the
    node in the ROS2 ecosystem. The name of the node must be unique in the ROS2 ecosystem. If two nodes have the
    same name, then the second node will not be able to start.
    
    The aim of this node will be to create a small node that will allow the bot to move around and navigate its
    space
    """
    def __init__(self) -> None:
        """
        The constructor of the class. It initializes the node with the name 'my_first_node'. In this method
        we also initialize the publishers and subscribers that we want to use in the node. As well as all the possible
        parameters that we want to be able to change from the launch file.
        """
        super().__init__('my_first_node')
        #self.position: PoseStamped = None
        
        # This is how you print to terminal in a ros node
        self.get_logger().info('FSM running')

        # Create a publisher for setting the position
        #self.position_publisher = self.create_publisher(
        #    PoseStamped,
        #    '/mineros/local_position/pose',
        #    10
        #)

        # Create a client for getting the bot position
        self.bot_pos_client = self.create_client(
            BotPos,
            '/mineros/get_bot_position',
        )

        self.set_position_client = self.create_client(
            MoveTo,
            '/mineros/set_position',
        )
        
        self.look_at_block_client = self.create_client(
            MoveTo,
            '/mineros/look_at_block',
        )

        self.block_info_client = self.create_client(
            BlockInfo,
            '/mineros/block_info'
        )

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )

        self.mine_block_client = self.create_client(
            MineBlock,
            '/mineros/mining/mine_block',
        )
        
        # Crafting
        self.craft_client = self.create_client(
            Craft,
            '/mineros/interaction/craft'
        )

        #Get inventory
        self.get_inventory_client = self.create_client(
            Inventory,
            '/mineros/inventory/content'
        )

        #Get recipe
        self.get_recipe_client = self.create_client(
            Recipe,
            '/mineros/interaction/recipe'
        )
                
        # Start the square movement test
        #self.move_in_square()
        # Start the crafting test
        self.getting_started()

    
    #def move_in_square(self):
    #    request = BotPos.Request()
    #    future = self.bot_pos_client.call_async(request)
    #    rclpy.spin_until_future_complete(self, future)
    #    initial_position = future.result().bot_pos
#
    #    if initial_position is None:
    #        self.get_logger().info('Failed to get initial position')
    #        return
    #
    #    initial_x = initial_position.pose.position.x
    #    initial_y = initial_position.pose.position.y
    #    initial_z = initial_position.pose.position.z
#
    #    # Define the square movement steps
    #    steps = [
    #        (initial_x + 10, initial_y, initial_z),
    #        (initial_x + 10, initial_y, initial_z + 10),
    #        (initial_x, initial_y, initial_z + 10),
    #        (initial_x, initial_y, initial_z)
    #    ]
#
    #    for step in steps:
    #        ps = PoseStamped()
    #        ps.pose.position.x = step[0]
    #        ps.pose.position.y = step[1]
    #        ps.pose.position.z = step[2]
#
    #        request = MoveTo.Request()
    #        request.pose = ps 
    #        self.set_position_client.wait_for_service()
    #        future = self.set_position_client.call_async(request)
    #        rclpy.spin_until_future_complete(self, future)



    # ------------------------------
    # Helper functions
    # ------------------------------
    
    def move_to(self, move_to_position):
        request = MoveTo.Request()
        request.pose = move_to_position

        while not self.set_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/set_position')
        self.set_position_client.wait_for_service()
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def get_position(self):
        request = BotPos.Request()

        while not self.bot_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/get_bot_position')
        future = self.bot_pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().bot_pos
    
    def craft_item(self, itemID, count, crafting_table=False):
        request = Craft.Request()
        request.item.id = itemID
        request.item.count = count
        if crafting_table:
            request.crafting_table = True
        
        while not self.craft_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/interaction/craft')
        future = self.craft_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        #self.get_logger().info(f'{future.result()}')

    def get_inventory(self):
        request = Inventory.Request()

        while not self.get_inventory_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/inventory/content')
        future = self.get_inventory_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        #self.get_logger().info(f'{future.result()}')
        return future.result()

    def get_recipe(self, itemID, count):
        request = Recipe.Request()
        request.item.id = itemID
        request.item.count = count

        while not self.get_recipe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/interaction/recipe')
        future = self.get_recipe_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        #self.get_logger().info(f'{future.result()}')
        return future.result().recipies

    def find_blocks(self, blockID, count, distance):
        request = FindBlocks.Request()
        request.blockid = blockID
        request.max_distance = distance
        request.count = count

        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/mining/find_blocks')
        future = self.find_block_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        #self.get_logger().info(f'{future.result()}')
        return future.result().blocks.poses

    def mine_block(self, blockPosition):
        req = MineBlock.Request()
        req.block.position = blockPosition
    
        while not self.mine_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/mining/mine_block')
        future = self.mine_block_client.call_async(req)
        self.get_logger().info(f'Mining block at: ({blockPosition.x}, {blockPosition.y}, {blockPosition.z})')
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def search_and_mine(self, blockID, count):
        blocks = self.find_blocks(blockID, count, 30)
        failed_mine_count = 0

        for block in blocks:
            #TODO: Check if tool is required, and if so, get the tool
            if not self.mine_block(block.position):
                #TODO: Mining sometimes fails, check why
                failed_mine_count += 1

        self.get_logger().info(f'Failed to mine: {failed_mine_count} blocks')
        blocks_mined = len(blocks) - failed_mine_count
        self.get_logger().info(f'Mined: {blocks_mined} blocks')

        if blocks_mined < count:
            self.get_logger().info('Not enough blocks mined, searching for more')
            current_position = self.get_position()
            new_position = current_position
            new_position.pose.position.x += 40
            self.move_to(new_position)
            self.search_and_mine(blockID, count - blocks_mined)

        
    # ------------------------------

    # Code for the basics of minecraft
    def getting_started(self):
        self.get_logger().info('Getting started')       

        # Mine a bunch of logs
        self.search_and_mine(46, 5)

        # Craft planks
        self.craft_item(5, 4)


        
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    