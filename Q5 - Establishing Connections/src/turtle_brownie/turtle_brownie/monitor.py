import rclpy
from rclpy.node import Node

from turtlesim.srv import Kill ## srv stands for service
from turtlesim.msg import Pose

from std_msgs.msg import String

import random
import math
import time

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.get_logger().info('monitor started')
        self.kill_publisher_ = self.create_publisher(String, '/kill_notify', 10)

        ## monitor turtles' positions and decide when to 'kill' them
        self.timer = self.create_timer(5.0, self.monitor_turtles)
        self.shredder_pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.shredder_pose_callback, 10)

        self.prey = {}

        self.kill_client = self.create_client(Kill, 'kill') ## 'kill' is the name of the service
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting again...')

    def shredder_pose_callback(self, msg):
        self.shredder_pose = msg

    def monitor_turtles(self):
        ## track shredder turtle position and orientation
        ## and positions of all prey turtles
        # shredder turtle

        self.prey['turtle1'] = (self.shredder_pose, time.time())
        for i in range(2, 5):  # assuming prey turtles are named turtle2, turtle3, ...
            

    def control_loop(self):
        if self.shredder_pose is None:
            return
        
        now = time.time()

        for name in list(self.prey.keys()):
            prey_pose, spawn_time = self.prey[name]
            elapsed = now - spawn_time
            if elapsed < 2.0:
                continue  # skip newly spawned turtles
            else:
                distance = math.sqrt((prey_pose.x - self.shredder_pose.x) ** 2 + (prey_pose.y - self.shredder_pose.y) ** 2)
                angle_to_prey = math.atan2(prey_pose.y - self.shredder_pose.y, prey_pose.x - self.shredder_pose.x)
                angle_diff = abs(angle_to_prey - self.shredder_pose.theta)
                if distance < 1.0 and angle_diff < (math.pi / 4):
                    # 'Kill' the prey turtle
                    request = Kill.Request()
                    request.name = name
                    future = self.kill_client.call_async(request)
                    future.add_done_callback(self.kill_response_callback)
                    del self.prey[name]

        
        ## if any prey turtle is within a certain distance and in front of shredder, 'kill' it




def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()
            
