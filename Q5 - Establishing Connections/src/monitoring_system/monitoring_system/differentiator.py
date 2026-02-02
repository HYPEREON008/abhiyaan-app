import rclpy
from rclpy.node import Node
import math
import time

from monitoring_sys_interfaces.msg import Coordinates
from std_msgs.msg import Float32

class differentiatorNode(Node):
    def __init__(self):
        super().__init__('differentiator')
        self.subscription_ = self.create_subscription(Coordinates, 'location', self.listener_callback, 10)
        ## creates a subscription to the 'location' topic
        self.publisher_ = self.create_publisher(Float32, 'speed', 10)

        self.history = []  ## to store previous coordinates and timestamps

        self.timer_ = self.create_timer(1.0, self.publish_speed) ## Timer to periodically publish speed

    def listener_callback(self, msg):
        now = time.time()
        self.history.append((now, msg.xcoord, msg.ycoord))

        ## Keep only the last 10 entries to limit memory usage
        if len(self.history) > 10:
            self.history.pop(0)

    def publish_speed(self):
        if len(self.history)<2:
            return  ## Not enough data to compute speed
        
        t1, x1, y1 = self.history[-2]
        t2, x2, y2 = self.history[-1]

        dx = x2 - x1
        dy = y2 - y1
        dt = t2 - t1
        
        if dt == 0: return ## Avoid division by zero
        speed = math.sqrt(dx*dx + dy*dy) / dt

        msg = Float32() ## Create a new Float32 message
        msg.data = speed ## Set the value

        self.publisher_.publish(msg) ## Publish the speed
        self.get_logger().info(f"Published Speed: {speed:.2f} units/s") ## Log the published speed
def main(args=None):
    print("starting differentiator node")
    rclpy.init(args=args) ## initialise the ROS 2 Python client library
    differentiator_node = differentiatorNode() ## Create an instance of the differentiatorNode
    rclpy.spin(differentiator_node)
    differentiator_node.destroy_node() ## Clean up and destroy the node
    rclpy.shutdown()





