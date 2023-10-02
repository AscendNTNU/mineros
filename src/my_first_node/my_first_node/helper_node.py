import rclpy
from rclpy.node import Node
from mineros_inter.srv import PlayerPose

from geometry_msgs.msg import PoseStamped

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
        
        # This is how you declare a subscription, the subscription listents to topics, when a message is
        # published to a topic the subcription runs the callback function
        self.local_position_sub = self.create_subscription(
            PoseStamped,                             # Message type
            '/mineros/local_position/pose',          # Topic address
            self.position_cb,                        # Callback function
            10                                       # The quality of service
        )
    
    def player_pose_service_callback(self, request, response):
        response.pose = self.current_position
        return response

    def position_cb(self, msg: PoseStamped):
        self.current_position = msg
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstHelperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    