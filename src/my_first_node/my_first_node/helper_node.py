import time
import rclpy
from rclpy.node import Node
from mineros_interface.srv import PlayerPose

from geometry_msgs.msg import PoseStamped
from mineros_interface.srv import BotPos

# Task 1:
# Create the required logic to allow your fsm node to request the current position of the bot using a service


class MyFirstHelperNode(Node):
    """
    This Node is technically useless and a terrible design pattern in real life. But for the sake of learning
    we have it here. This node is already set up to listen to the current position of the bot.
    """

    def __init__(self) -> None:
        
        super().__init__('helper_node')
        self.get_logger().info('Helper running')
        self.get_player_pose = self.create_service(PlayerPose,'player_pose_client',self.player_pose_service_callback)
        self.current_position = None
        
        cb_group = ReentrantCallbackGroup()
        # This is how you declare a subscription, the subscription listents to topics, when a message is
        # published to a topic the subcription runs the callback function
        self.local_position_sub = self.create_subscription(
            PoseStamped,                             # Message type
            '/mineros/local_position/pose',          # Topic address
            self.position_cb,                        # Callback function
            10,
            callback_group=cb_group
        )
        
        self.bot_pos_service = self.create_service(
            BotPos,
            '/mineros/get_bot_position',
            self.get_bot_position_cb,
            callback_group=cb_group
        )
    
    def player_pose_service_callback(self, request, response):
        response.pose = self.current_position
        return response

    def position_cb(self, msg: PoseStamped):
        self.current_position = msg
        
    def get_bot_position_cb(self, request, response):
        while self.current_position is None:
            self.get_logger().info('Waiting for position')
            time.sleep(0.1)
            
        response.bot_pos = self.current_position
        return response
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstHelperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    