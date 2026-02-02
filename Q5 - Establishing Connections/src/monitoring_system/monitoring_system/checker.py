import rclpy
from rclpy.node import Node

from monitoring_sys_interfaces.msg import Coordinates
from std_msgs.msg import Float32, String

class checkerNode(Node):
    def __init__(self):


        super().__init__('checker')

        self.speed_limit = 1.5  ## Define speed limit

        self.subscription_ = self.create_subscription(Float32, 'speed', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'warnings', 10)
        
    def listener_callback(self, msg):
        speed = msg.data
        if speed > self.speed_limit:
            reduction = speed - self.speed_limit
            
            warning_text = (f"Speed limit exceeded, please reduce the speed by {reduction:.2f}.")

            warning_msg = String()
            warning_msg.data = warning_text
            self.publisher_.publish(warning_msg)
            self.get_logger().info(warning_text)

def main(args=None):
    print("starting checker node")
    rclpy.init(args=args)  ## initialise the ROS 2 Python client library
    checker_node = checkerNode()  ## Create an instance of the checkerNode
    rclpy.spin(checker_node)
    checker_node.destroy_node()  ## Clean up and destroy the node
    rclpy.shutdown()
