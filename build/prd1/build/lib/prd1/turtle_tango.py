import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn
from time import sleep
from math import radians

class Obroty(Node):
    def __init__(self):
        super().__init__("skz")

        self.rotate_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')
        self.spawn_client = self.create_client(Spawn, 'spawn')

        self.declare_parameter(name='turtle2')
        self.declare_parameter(name='turtle3')

        self.in_action = False

        while not self.rotate_client.wait_for_server(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')


    def rotate_first_turtle(self, angle):

        self.in_action = True

        angle2 = radians(angle)
        goal = RotateAbsolute.Goal(theta=angle2)

        self.get_logger().info(f'Rotating by {angle} degrees')
        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def spawn_turtle(self, i):

        self.in_action = True

        my_param = self.get_parameter(f'turtle{i}').get_parameter_value().string_value

        self.message = Spawn.Request()
        self.message.name = my_param
        self.message.x = 5.44 - float(i)
        self.message.y = float(i*2)
 
        self.get_logger().info(f'Spawning turtle number {i} - {self.message.name}')
        future = self.spawn_client.call_async(self.message)
        self.in_action = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.in_action = False

def main(args=None):
    rclpy.init(args=args)

    client = Obroty()

    client.rotate_first_turtle(-180)
    while client.in_action: 
        rclpy.spin_once(client)
    sleep(1)

    client.spawn_turtle(2)
    while client.in_action: 
        rclpy.spin_once(client)
    sleep(1)

    client.rotate_first_turtle(5)
    while client.in_action:
        rclpy.spin_once(client)
    client.rotate_first_turtle(270)
    while client.in_action:
        rclpy.spin_once(client)
    sleep(1)

    client.spawn_turtle(3)
    while client.in_action: 
        rclpy.spin_once(client)

    rclpy.shutdown()



if __name__ == '__main__':
    main()

        



        
