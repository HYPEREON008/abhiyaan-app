import rclpy
from rclpy.node import Node ## Importing Node class from rclpy.node module
## rclpy is ROS2 client library for Python
from geometry_msgs.msg import Twist ## Importing Twist from msg module of geometry_msgs package, Twise message type gives velocity broken into linear and angular parts
import math


class DrawCircle(Node):
    def __init__(self):
        super().__init__('draw_circle') ## Initializing the Node with the name 'draw_circle'
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) ## Creating a publisher to publish Twist messages to 'turtle1/cmd_vel' topic with a queue size of 10
        self.timer = self.create_timer(0.1, self.move_circle) ## Creating a timer that calls move_circle method every 0.1 second

        self.linear_speed = 2.0
        self.angular_speed = 1.0

        ## why call every 0.1 second? because In ROS, constant motion requires constant publishing of velocity commands.
    def move_circle(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    print("start")
    rclpy.init(args=args)
    node = DrawCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("end")

if __name__ == '__main__':
    main()
