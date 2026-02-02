import rclpy ## ROS 2 Python client library
from rclpy.node import Node
import random

from monitoring_sys_interfaces.msg import Coordinates

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps')
        self.publisher_ = self.create_publisher(Coordinates, 'location', 10)
        ## publishes to the 'location' topic which uses the Coordinates message type

        self.timer_ = self.create_timer(0.25, self.publish_location) ## Publish every second
        ## in Python, methods can be referenced before they are defined within the same class.

    def publish_location(self):
        msg = Coordinates() ## Create a new Coordinates message
        msg.xcoord = random.uniform(-10.0, 10.0) ## Simulate x coordinate
        msg.ycoord = random.uniform(-10.0, 10.0) ## Simulate y coordinate
        self.publisher_.publish(msg) ## Publish the message
        self.get_logger().info(f'Published GPS Coordinates: x={msg.xcoord:.2f}, y={msg.ycoord:.2f}') ## Log the published coordinates
        ## self.get_logger() creates a logger instance for logging messages
def main(args=None):
    print("starting gps node")
    rclpy.init(args=args) ## Initialise the ROS 2 Python client library
    gps_node = GPSNode() ## Create an instance of the GPSNode
    rclpy.spin(gps_node) 
    gps_node.destroy_node() ## Clean up and destroy the node
    rclpy.shutdown()
        
