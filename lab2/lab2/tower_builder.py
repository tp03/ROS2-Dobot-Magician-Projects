import rclpy
import rclpy.node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from time import sleep


fixed_x = 160
fixed_y = 40
tower_x = 220
tower_y = 0

class Pick_place(rclpy.node.Node):
    def __init__(self):
        super().__init__('tower_builder')
        
        self.cube_height = 20
        
        self.declare_parameter(name='tower_size')
        self.tower_size = self.get_parameter('tower_size').get_parameter_value().integer_value

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

    def tower(self):
        for i in range(self.tower_size):
            self.change_holding(True)
            while self.in_action == True:
                rclpy.spin_once(self)
            self.jump([fixed_x + 40*i, fixed_y, 80])
            while self.in_action == True:
                rclpy.spin_once(self)
            self.jump([fixed_x + 40*i, fixed_y, 2])
            while self.in_action == True:
                rclpy.spin_once(self)
            self.change_holding(False)
            while self.in_action == True:
                rclpy.spin_once(self)
            self.jump([fixed_x + 40*i, fixed_y, 80])
            while self.in_action == True:
                rclpy.spin_once(self)
            self.jump([tower_x, tower_y, i*20+10])
            while self.in_action == True:
                rclpy.spin_once(self)
            self.jump([tower_x, tower_y, 2 + i*20])
            while self.in_action == True:
                rclpy.spin_once(self)    
        self.change_holding(True)
        while self.in_action == True:
            rclpy.spin_once(self)
        self.jump([tower_x, tower_y, 80])
        while self.in_action == True:
            rclpy.spin_once(self)

def main():
    rclpy.init()
    client = Pick_place()
    client.tower()
    rclpy.shutdown()

if __name__ == '__main__':
    main()