import rclpy
from rclpy.node import Node ## Importing Node class from rclpy.node module
## rclpy is ROS2 client library for Python
from geometry_msgs.msg import Twist ## Importing Twist from msg module of geometry_msgs package, Twise message type gives velocity broken into linear and angular parts
import math

class drawSquare(Node):
    def __init__(self):
        super().__init__('draw_square') ## Initializing the Node with the name 'draw_square'
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) ## Creating a publisher to publish Twist messages to 'turtle1/cmd_vel' topic with a queue size of 10

        self.linear_speed = 2.0 ## Linear speed of the turtle
        self.angular_speed = math.pi / 2  ## Angular speed of the turtle 90 deg / sec

        self.state = 0
        self.timer = self.create_timer(1.0, self.tick)
        ## moves straight for 1 second and turns for 1 second alternately to draw a square

    def tick(self):
        if self.state == 0:
            self.straight()
        else:
            self.turn()
        self.state ^= 1 ## ^ is bitwise XOR operator, toggles between 0 and 1

    def straight(self):
        msg = Twist() ## Creating a Twist message
        msg.linear.x = self.linear_speed ## Setting linear velocity in x direction
        self.publisher_.publish(msg) ## Publishing the message
       
    def turn(self):
        msg = Twist() ## Creating a Twist message
        msg.angular.z = self.angular_speed ## Setting angular velocity around z-axis
        self.publisher_.publish(msg) ## Publishing the message
    
def main(args=None):
    print("start")
    rclpy.init(args=args) ## Initializing rclpy
    node = drawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("end")



if __name__=='__main__':
    main()

