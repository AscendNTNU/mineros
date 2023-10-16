import rclpy
import subprocess
import os
import signal

from rclpy.node import Node
from std_srvs.srv import Trigger 



class WatchdogNode(Node):
    
    def __init__(self) -> None:
        
        super().__init__('watchddog_node')
        self.restart_mineros_service = self.create_service(
            Trigger,
            '/mineros/reboot',
            self.restart_mineros_cb
        )
            
    def restart_mineros_cb(self, request, response):
        FIND_NODE_PID = """ps aux | grep "/opt/ros/foxy/bin/ros2 run mineros mineros_main" | grep -v grep | awk '{print $2}'"""
        RUN_MINE_ROS = """ros2 run mineros mineros_main"""
        try:
            node_pid = self.do_command_with_output(FIND_NODE_PID)
            os.kill(node_pid, signal.SIGKILL)
            self.do_command(RUN_MINE_ROS)
        except Exception as e:
            self.get_logger().error(f'{e}')
            return False
    
    def do_command_with_output(self, command: str):
        output = subprocess.check_output(command, shell=True, universal_newlines=True)
        return int(output)

    def do_command(self, command: str):
        subprocess.Popen(command.split(), stdout=subprocess.PIPE)

    
def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    