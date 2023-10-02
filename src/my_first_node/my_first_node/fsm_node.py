import time
import rclpy
from rclpy.node import Node

from typing import Any
from geometry_msgs.msg import PoseStamped
from mineros_interface.srv import BlockInfo
from mineros_interface.srv import PlayerPose
from std_msgs.msg import String
import copy

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
        
        # This is how you print to terminal in a ros node
        self.get_logger().info('Hello World!')
        
        # This definition isnt needed for this example, it only shows how to implement a client, we wil only be using publishers
        # and subscribers for this tutorial. Implementing publishers and subscribers is similar
        self.find_y_client = self.create_client(
            BlockInfo,
            '/mineros/findy'
        )
        # Task 2
        # To make the bot move in a square we need to know the bots location and we need to be able to move the bot
        # set up the required publisher and subscriber here:
        self.pose_client = self.create_client(PlayerPose, 'player_pose_client')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pose_request = PlayerPose.Request()

        self.pose_publisher = self.create_publisher(PoseStamped,'/mineros/set_position',10)
        #self.pose_subscription = self.create_subscription(String,'/mineros/set_position',self.move_to_pose,10)
    
        current_position: PoseStamped = self.send_pose_request().pose
        current_position.pose.position
        goal_position = copy.deepcopy(current_position)
        goal_position.pose.position.x = -21.0
        goal_position.pose.position.y = 122.0
        goal_position.pose.position.z = -39.0
        self.move_to_pose(goal_position)
    #
    def send_pose_request(self):
        self.future = self.pose_client.call_async(self.pose_request)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

    def move_to_pose(self, msg):
        self.pose_publisher.publish(msg)    

           
            
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    