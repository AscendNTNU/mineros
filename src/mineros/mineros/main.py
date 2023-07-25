import time
from threading import Thread
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


from mavros_msgs.srv import SetMode

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from mineros_interfaces.srv import FindBlocks, MineBlocks, Inventory, PlaceBlocks
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

        self.mode = 'Loiter'
        self.mission = []
        self.mission_thread = None
        self.current_position = self.bot.entity.position

        timers_cbg = ReentrantCallbackGroup()

        self.mode_control_service = self.create_service(
            SetMode,
            '/mineros/set_mode',
            self.mode_control_callback
        )

        # Movement control
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

        self.mine_blocks_service = self.create_service(
            MineBlocks,
            '/mineros/mining/mine_blocks',
            self.mine_blocks_callback
        )

        self.inventory_contents_service = self.create_service(
            Inventory,
            '/mineros/inventory/contents',
            self.inventory_contents_service_callback
        )

        # Interaction control
        self.place_blocks_service = self.create_service(
            PlaceBlocks,
            '/mineros/interaction/place_blocks',
            self.place_blocks_service_callback
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

    def mode_control_callback(self, request: SetMode.Request, response: SetMode.Response):
        self.get_logger().info(f"Mode control: {request.custom_mode}")
        self.mode = request.custom_mode
        response.success = True
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

    def mine_blocks_callback(self, request: MineBlocks.Request, response: MineBlocks.Response):
        poses: List[Pose] = request.blocks.poses

        if len(poses) == 0:
            response.success = False
            return response

        success = []
        for pose in poses:
            vec = Vec3(pose.position.x, pose.position.y, pose.position.z)

            block = self.bot.blockAt(vec)
            if not self.bot.canDigBlock(block):
                self.get_logger().info(f"Can't dig block: {vec}")
                success.append(False)

            self.bot.collectBlock.collect(block)

            self.get_logger().info(f"collected block: {vec}")
            success.append(True)

        response.success = success
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

    def place_blocks_service_callback(self, request: PlaceBlocks.Request, response: PlaceBlocks.Response):
        blocks: List[BlockPose] = request.blocks

        self.get_logger().info(f"Placing {len(blocks)} blocks")

        if len(blocks) == 0:
            response.success = [False for _ in range(len(blocks))]
            return response

        response.success = []
        for block in blocks:
            item: Item = block.block
            point: Point = block.point
            face_vector: Point = block.face_vector
            point = Vec3(point.x, point.y, point.z)
            face_vector = Vec3(face_vector.x, face_vector.y, face_vector.z)
            
            self.get_logger().info(f"Placing block: {item.id} at {point} with {face_vector}")

            block = self.bot.blockAt(point)

            if block.name == 'air':
                self.get_logger().info(f"Can't place block on air")
                response.success.append(False)
                continue

            # Get to block
            self.bot.pathfinder.setGoal(None)
            # goal = pathfinder.goals.GoalGetToBlock(point.x - 1, point.y, point.z)
            goal = pathfinder.goals.GoalPlaceBlock(block.position.plus(face_vector), self.bot.world, {'range':5, 'half': 'top'})
            self.bot.pathfinder.setGoal(goal)

            while self.bot.pathfinder.isMoving():
                time.sleep(0.5)

            # Bad hack to get the item object TODO: Fix this, if problem
            items = self.bot.inventory.items()
            item = list(filter(lambda i: i.type == item.id, items))[0]

            self.bot.equip(item, 'hand')


            # THIS IS VERY WEIRD BUT GOD KNOWS HOW THIS SHITTY JAVASCRIPT GARBAGE WORKS
            # WHAT THE FUCK
            
            # It seems that the problem has to do with timing, hence the 1 second sleep
            # the nested try catch is in case the 1 second sleep is too short, the try catch
            # works because whenever it fails it incours a 5 second timeout this shitty structure 
            # causes the 
            try:
                time.sleep(1)
                self.bot.placeBlock(block, face_vector)
                response.success.append(True)
            except Exception as e:
                self.get_logger().info(f"Failed to place block on first try. retrying")
                try:
                    self.bot.placeBlock(block, face_vector)
                    response.success.append(True)
                except Exception as e:
                    self.get_logger().info(f'{e}')
                    response.success.append(False)
            
            

        return response

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
