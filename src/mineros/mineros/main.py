import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State

from geometry_msgs.msg import PoseStamped, PoseArray, Pose

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')


class MinerosMain(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        # Params
        self.declare_parameter('goal_acceptance', 1)
        self.declare_parameter('bot_username', 'MinerosBot')
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

        # Mode control
        """
        Modes: 
        - Loiter
        - Active
        """

        self.mode_control_service = self.create_service(
            SetMode,
            '/mineros/set_mode',
            self.mode_control_callback
        )

        # Active mode
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

        # Info Publishers

        # State publishers
        self.state_topic = self.create_publisher(
            State,
            '/mineros/state',
            10
        )

        self.state_timer = self.create_timer(
            0.2,
            self.state_timer_callback,
            callback_group=timers_cbg
        )

        # Active mode publishers
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

    def state_timer_callback(self):
        state = State()
        state.mode = self.mode
        self.state_topic.publish(state)

    def local_pose_timer_callback(self):
        bot_position = self.bot.entity.position
        self.get_logger().info(f"Bot position: {bot_position}")

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
