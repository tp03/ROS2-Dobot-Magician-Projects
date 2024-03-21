import rclpy
import rclpy.node
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn
from rclpy.action import ActionClient
from math import radians
from time import sleep

class TurtleNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('turtle_node')
        self.in_action = False
        self.rotate_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')
        self.spawn_client= self.create_client(Spawn, 'spawn')
        self.declare_parameter(name='turtle2name')
        self.declare_parameter(name='turtle3name')

        while not self.rotate_client.wait_for_server(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')

    def timer_callback(self):
        pass

    
    def rotate(self, id, angle):
        self.in_action = True
        angle2 = radians(angle)
        goal = RotateAbsolute.Goal(theta=angle2)
        self.get_logger().info(f'Rotating by {angle} degrees')
        future = self.rotate_client.send_goal_async(goal)

        future.add_done_callback(self.goal_response_callback)

        # self.in_action = False

    def spawn(self, id, pos = [05.44, 5]):
        self.in_action = True
        my_param = self.get_parameter(f'turtle{id}name').get_parameter_value().string_value
        self.message = Spawn.Request()
        self.message.name = my_param
        self.message.x = float(pos[0])
        self.message.y = float(pos[1])
        self.get_logger().info(f'Spawning turtle number {id} - {self.message.name}')
        future = self.spawn_client.call_async(self.message)
        self.in_action = False
    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.in_action = False


def main():
    rclpy.init()
    node = TurtleNode()
    node.rotate(1, -180)
    while node.in_action: 
        rclpy.spin_once(node)
    sleep(1)
    node.spawn(2)
    while node.in_action: 
        rclpy.spin_once(node)
    sleep(1)
    node.rotate(1, 5)
    while node.in_action: 
        rclpy.spin_once(node)
    # sleep(1)
    node.rotate(1, -90)
    while node.in_action: 
        rclpy.spin_once(node)
    sleep(1)
    node.spawn(3)
    while node.in_action: 
        rclpy.spin_once(node)
    sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()