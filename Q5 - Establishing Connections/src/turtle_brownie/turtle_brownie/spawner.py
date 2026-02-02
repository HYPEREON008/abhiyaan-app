import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn ## srv stands for service
from std_msgs.msg import String

import random
import math

class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.get_logger().info('spawner started')

        self.max_prey = 3
        self.active_turtles = []

        self.kill_subscription_ = self.create_subscription(String, '/kill_notify', self.kill_callback, 10)

        ## when topic names have a leading '/', they are considered global topics
        ## without leading '/', they are relative to the node's namespace

        # Create a client for the Spawn service
        self.spawn_client = self.create_client(Spawn, 'spawn') ## 'spawn' is the name of the service
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        
        ## spawn initial cluster of turtles
        for i in range(self.max_prey):
            self.spawn_turtle()
    
    def spawn_turtle(self):
        if len(self.active_turtles) < self.max_prey:
            ## Create a request object for the Spawn service
            ## Spawn srv is defined in turtlesim/srv/Spawn.srv
            request = Spawn.Request()
            request.x = random.uniform(1.0, 10.0)
            request.y = random.uniform(1.0, 10.0)
            request.theta = random.uniform(0, 2 * math.pi)

            ## Call the Spawn service asynchronously
            future = self.spawn_client.call_async(request) ## returns a future obj that will hold the response
            future.add_done_callback(self.spawn_response_callback) ## when the response is ready, call this function

            self.spawn_notify_publisher = self.create_publisher(String, '/spawn_notify', 10)
    
    def spawn_response_callback(self, future):
        try:
            response = future.result()
            turtle_name = response.name
            self.active_turtles.append(turtle_name)
            self.get_logger().info(f"Spawned turtle: {turtle_name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def kill_callback(self, turtle_name):
        if turtle_name in self.active_turtles:
            self.active_turtles.remove(turtle_name)
            self.get_logger().info(f"Turtle {turtle_name} removed from active list.")
        ## spawn a new turtle to maintain the population
        self.spawn_turtle()

def main(args=None):
    rclpy.init(args=args)
    spawner = Spawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()
            
