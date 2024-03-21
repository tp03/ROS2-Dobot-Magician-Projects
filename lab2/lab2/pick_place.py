import rclpy
import rclpy.node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from time import sleep

class Pick_place(rclpy.node.Node):
    def __init__(self):
        super().__init__('pick_place')

        self.base_x = 0
        self.base_y = 0
        self.base_z = 0

        self.cube_x = 0
        self.cube_y = 0
        self.cube_z = 0

        self.save_z = 80
        self.cube_height = 30

        self.move_client = ActionClient(self, PointToPoint, '/PTP_action')

        self.grap_client = self.create_client(GripperControl, '/dobot_gripper_service')



    def jump(self, pos):
        sleep(1)
        final_x = pos[0]
        final_y = pos[1]
        final_z = pos[2]


        self.in_action = True

        move_request = PointToPoint.Goal()
        move_request.target_pose = [float(final_x), float(final_y), float(final_z), 0.0]
        move_request.motion_type = 1

        self.get_logger().info("knvioerhg")

        future = self.move_client.send_goal_async(move_request)
        future.add_done_callback(self.goal_response_callback)

        # self.in_action = False
    def change_holding(self, state):
        sleep(1)
        self.in_action = True

        request = GripperControl.Request()
        request.keep_compressor_running = False

        if state:
            request.gripper_state = "open"
        else:
            request.gripper_state = "close"

        future = self.grap_client.call_async(request)
        # future.add_done_callback(self.goal_response_callback)

        self.in_action = False
    
    #possible to use

    def set_base_position(self, x, y, z):
        self.base_x = x
        self.base_y = y
        self.base_z = z

    def set_first_cube_position(self, x, y, z):
        self.cube_x = x
        self.cube_y = y
        self.cube_y = z
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.in_action = False

def main():
    rclpy.init()
    client = Pick_place()
    client.change_holding(True)
    while client.in_action == True:
        rclpy.spin_once(client)
    client.jump([200.0, 20.0, 10.0])
    while client.in_action == True:
        rclpy.spin_once(client)
    client.get_logger().info("shsujydcuye")
    client.change_holding(False)
    while client.in_action == True:
        rclpy.spin_once(client)
    client.jump([200.0, 20.0, 60.0])
    while client.in_action == True:
        rclpy.spin_once(client)
    client.jump([200.0, 60.0, 60.0])
    while client.in_action == True:
        rclpy.spin_once(client)
    client.jump([200.0, 60.0, 10.0])
    while client.in_action == True:
        rclpy.spin_once(client)
    client.change_holding(True)
    while client.in_action == True:
        rclpy.spin_once(client)
    #client.set_base_position(30, 30, 30)
    #client.set_first_cube_position(60, 60, 30)
    rclpy.shutdown()

if __name__ == '__main__':
    main()