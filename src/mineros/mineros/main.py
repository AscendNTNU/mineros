import time
from threading import Thread
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from mineros_interfaces.srv import FindBlocks, MineBlock, Inventory, PlaceBlock, Craft, BlockInfo
from mineros_interfaces.msg import Item, BlockPose

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')
toolPlugin = require('mineflayer-tool').plugin
collectBlock = require('mineflayer-collectblock').plugin
Vec3 = require('vec3').Vec3


class MinerosMain(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        # Params
        self.declare_parameter('goal_acceptance', 1)
        self.declare_parameter('bot_username', 'MinerosBotTest')
        self.declare_parameter('lan_port', 25565)

        BOT_USERNAME = self.get_parameter(
            'bot_username').get_parameter_value().string_value
        LAN_PORT = self.get_parameter(
            'lan_port').get_parameter_value().integer_value
        self.goal_acceptance = self.get_parameter(
            'goal_acceptance').get_parameter_value().double_value

        self.bot = mineflayer.createBot(
            {'host': 'localhost', 'port': LAN_PORT, 'username': BOT_USERNAME, 'hideErrors': False})
        self.bot.loadPlugin(pathfinder.pathfinder)
        self.bot.loadPlugin(toolPlugin)
        self.bot.loadPlugin(collectBlock)

        # The spawn event
        once(self.bot, 'login')
        self.bot.chat('I spawned')

        # Instantiating pathfinding
        self.movements = pathfinder.Movements(self.bot)
        self.bot.pathfinder.setMovements(self.movements)

        self.mission = []
        self.mission_thread = None
        self.current_position = self.bot.entity.position

        timers_cbg = ReentrantCallbackGroup()

        #Movement control
        self.find_y_service = self.create_service(
            BlockInfo,
            '/mineros/findy',
            self.find_y_callback
        )

        self.set_potision = self.create_subscription(
            PoseStamped,
            '/mineros/set_position',
            self.set_position_callback,
            10
        )

        self.set_position_composite = self.create_subscription(
            PoseArray,
            '/mineros/set_position/composite',
            self.set_position_composite_callback,
            10
        )

        # Mining control
        self.find_blocks_service = self.create_service(
            FindBlocks,
            '/mineros/mining/find_blocks',
            self.find_blocks_callback
        )

        self.mine_block_service = self.create_service(
            MineBlock,
            '/mineros/mining/mine_block',
            self.mine_block_callback
        )

        self.inventory_contents_service = self.create_service(
            Inventory,
            '/mineros/inventory/contents',
            self.inventory_contents_service_callback
        )

        # Interaction control
        self.place_block_service = self.create_service(
            PlaceBlock,
            '/mineros/interaction/place_block',
            self.place_block_service_callback
        )

        self.craft_item_service = self.create_service(
            Craft,
            '/mineros/interaction/craft_item',
            self.craft_item_service_callback
        )

        # Info publishers
        self.local_position_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/local_position/pose',
            10
        )

        self.local_pose_timer = self.create_timer(
            0.1,
            self.local_pose_timer_callback,
            callback_group=timers_cbg
        )

    def find_y_callback(self, request: BlockInfo, response: BlockInfo):
        """Returns the ground level y coordinate for a given x and z coordinate"""
        self.get_logger().info("find_y_callback")
        
        response.block = BlockPose()
        response.block.block_pose = Pose()
        response.block.block_pose.position.x = request.block_pose.position.x
        response.block.block_pose.position.z = request.block_pose.position.z
        self.get_logger().info(f'find_y_callback: {request.block_pose.position.x}, {request.block_pose.position.z}')
        
        bot_position = self.bot.entity.position
        block_at_bot_height = self.bot.blockAt(Vec3(request.block_pose.position.x, bot_position.y, request.block_pose.position.z))
        
        iteration = 1
        if block_at_bot_height.type == 0:
            iteration = -1

        block = block_at_bot_height
        while True:
            self.get_logger().info(f'{block.type} , {block.name}, {block.position.y}, {iteration}')
            previous_block = block
            new_pose = Vec3(request.block_pose.position.x, previous_block.position.y + iteration, request.block_pose.position.z)
            block = self.bot.blockAt(new_pose)
            
            # Iterated from dirt to sky
            if block.type == 0 and previous_block.type != 0:
                self.get_logger().info(f'Found y: {block.type}, {block.position.y}, {iteration}')
                response.block.block_pose.position.y = float(previous_block.position.y)
                return response
            # from sky to dirt
            elif block.type != 0 and previous_block.type == 0:
                self.get_logger().info(f'Found y: {previous_block.type}, {previous_block.position.y}, {iteration}')
                response.block.block_pose.position.y = float(block.position.y)
                return response
            

    def set_position_callback(self, msg: PoseStamped):
        self.bot.pathfinder.setGoal(None)

        if msg.pose.position.y == -1.0:
            goal = pathfinder.goals.GoalNearXZ(
                round(msg.pose.position.x, 2), round(msg.pose.position.z, 2), self.goal_acceptance)
        else:
            goal = pathfinder.goals.GoalNear(
                round(msg.pose.position.x, 2), round(msg.pose.position.y, 2), round(msg.pose.position.z, 2), self.goal_acceptance)

        self.bot.pathfinder.setGoal(goal)

    def set_position_composite_callback(self, msg: PoseArray):
        self.bot.pathfinder.setGoal(None)

        goals = []
        for pose in msg.poses:

            if pose.position.y == -1.0:
                goals.append(pathfinder.goals.GoalNearXZ(
                    round(pose.position.x, 2), round(pose.position.z, 2), self.goal_acceptance))
            else:
                goals.append(pathfinder.goals.GoalNear(
                    round(pose.position.x, 2), round(pose.position.y, 2), round(pose.position.z, 2), self.goal_acceptance))

        goal = pathfinder.goals.GoalCompositeAll(goals)
        self.bot.pathfinder.setGoal(goal)

    def find_blocks_callback(self, request: FindBlocks.Request, response: FindBlocks.Response):
        self.get_logger().info(f"Find blocks: {request.blockid}")

        options = {}
        options['matching'] = request.blockid
        if request.max_distance != 0:
            options['maxDistance'] = request.max_distance
        if request.count != 0:
            options['count'] = request.count

        blocks = self.bot.findBlocks(options)

        assert blocks is not None

        pa = PoseArray()
        for block in blocks:
            p = Pose()
            p.position.x = float(block.x)
            p.position.y = float(block.y)
            p.position.z = float(block.z)
            pa.poses.append(p)

        response.blocks = pa
        return response

    def mine_block_callback(self, request: MineBlock.Request, response: MineBlock.Response):
        pose: Pose = request.block

        vec = Vec3(pose.position.x, pose.position.y, pose.position.z)
        block = self.bot.blockAt(vec)

        # if not self.bot.canDigBlock(block):
        #     self.get_logger().info(f"Can't dig block: {vec}")
        #     self.get_logger().info(f"Block: {block}")
        #     response.success = False
        #     return response

        self.bot.collectBlock.collect(block, timeout=10)
        self.get_logger().info(f"collected block: {vec}")

        response.success = True
        return response

    def inventory_contents_service_callback(self, request: Inventory.Request, response: Inventory.Response):
        items = self.bot.inventory.items()
        inventory = []

        self.get_logger().info(f"Inventory contents: {items}")
        for item in items:
            item_msg = Item()
            item_msg.id = item.type
            item_msg.count = item.count
            item_msg.slot = item.slot
            item_msg.metadata = item.metadata
            inventory.append(item_msg)

        response.inventory = inventory
        return response

    def place_block_service_callback(self, request: PlaceBlock.Request, response: PlaceBlock.Response):
        block: BlockPose = request.block

        self.get_logger().info(f"Placing {block} ")

        item: Item = block.block
        block_pose: Pose = block.block_pose
        point: Point = block_pose.position
        face_vector: Point = block.face_vector
        point = Vec3(point.x, point.y, point.z)
        face_vector = Vec3(face_vector.x, face_vector.y, face_vector.z)

        self.get_logger().info(
            f"Placing block: {item.id} at {point} with {face_vector}")
        block = self.bot.blockAt(point)

        if block.name == 'air':
            self.get_logger().info(f"Can't place block on air")
            response.success = False
            return response

        # Get to block
        self.bot.pathfinder.setGoal(None)
        
        goal = pathfinder.goals.GoalPlaceBlock(block.position.plus(
            face_vector), self.bot.world, {'range': 5, 'half': 'top'})
        self.bot.pathfinder.setGoal(goal)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        while self.bot.pathfinder.isMoving():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        items = self.bot.inventory.items()
        item = list(filter(lambda i: i.type == item.id, items))[0]
        self.bot.equip(item, 'hand')

        placed = False
        while not placed:
            try:
                rclpy.spin_once(self, timeout_sec=2)
                self.bot.placeBlock(block, face_vector)
                placed = True
                response.success = True
            except Exception as e:
                
                self.get_logger().info(f"{e}")
                
            
        self.get_logger().info(f"Placed block: {item.id} at {point} with {face_vector}")
        
        response.success = True

    def craft_item_service_callback(self, request: Craft.Request, response: Craft.Response):
        item = request.item

        if request.crafting_table:
            crafting_table = self.bot.blockAt(Vec3(
                request.crafting_table_location.x, request.crafting_table_location.y, request.crafting_table_location.z))
        else:
            crafting_table = None  # None is an alias for the player's inventory

        recipe = self.bot.recipesFor(item.id, None, item.count, crafting_table)

        if len(recipe) == 0:
            self.get_logger().info(f"Can't craft item: {item.id}")
            response.success = False
            return response

        self.bot.craft(recipe[0], item.count, crafting_table)
        self.get_logger().info(f"Crafted item: {item.id}")
        response.success = True

        return response

    # TODO use stove

    # TODO kill or avoid mobs

    def local_pose_timer_callback(self):
        bot_position = self.bot.entity.position
        # self.get_logger().info(f"Bot position: {bot_position}")

        pose = PoseStamped()
        pose.pose.position.x = float(bot_position.x)
        pose.pose.position.y = float(bot_position.y)
        pose.pose.position.z = float(bot_position.z)
        self.local_position_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MinerosMain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
