import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from ros2_aruco_interfaces.msg import ArucoMarkers
from transforms3d.euler import quat2euler, euler2quat
from std_msgs.msg import Bool
from numpy import sin, cos
import numpy as np
from time import sleep

camera_effector_difference_x = 0.04
camera_effector_difference_y = 0.07
camera_effector_difference_z = 0.06
fr = 10

class FinalBoss(Node):
    def __init__(self):
        super().__init__('final_boss')
        self.move_client = ActionClient(self, PointToPoint, '/PTP_action')
        self.grap_client = self.create_client(GripperControl, '/dobot_gripper_service')
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.m_subscriber = self.create_subscription(ArucoMarkers, 'aruco_markers', self.pose_callback, 10)   
        self.flag_publisher = self.create_publisher(Bool, 'important_flag', 10)

        self.angle_1 = 0.0
        self.point_1 = None
        self.point_3 = None

    def joint_states_callback(self ,msg: JointState):
        theta_vector = msg.position
        joint_count = len(theta_vector)

        self.angle1 = theta_vector[0]

        matrixes = []

        matrixes.append(np.matrix([[cos(theta_vector[0]), -sin(theta_vector[0]), 0, 0],
                                   [sin(theta_vector[0]), cos(theta_vector[0]), 0, 0],
                                   [0, 0, 1, 0.05],
                                   [0, 0, 0, 1]                                   
                                   ]))
        
        matrixes.append(np.matrix([
                    [cos(theta_vector[1]), 0, sin(theta_vector[1]), 0],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[1]), 0, cos(theta_vector[1]), 0.088],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(theta_vector[2]), 0, sin(theta_vector[2]), 0],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[2]), 0, cos(theta_vector[2]), 0.135],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(theta_vector[3]), 0, sin(theta_vector[3]), 0.147],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[3]), 0, cos(theta_vector[3]), 0],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([[cos(theta_vector[4]), -sin(theta_vector[4]), 0, 0.03],
                                   [sin(theta_vector[4]), cos(theta_vector[4]), 0, 0],
                                   [0, 0, 1, -0.024-0.04-0.005],
                                   [0, 0, 0, 1]                                   
                                   ]))

        matrixes.append(np.matrix([
                    [1, 0, 0, 0],
                    [0, cos(np.pi), -sin(np.pi), 0],
                    [0, sin(np.pi), cos(np.pi), 0],
                    [0, 0, 0, 1]
        ]))
        

        for i in range(joint_count):
            R = np.matmul(matrixes[i], matrixes[i+1])
            matrixes[i+1] = R

        self.tab = matrixes[5]        
        self.tbc = np.matrix([
            [-1, 0, 0, camera_effector_difference_x],
            [0, -1, 0, camera_effector_difference_y],
            [0, 0, 1, -camera_effector_difference_z],
            [0, 0, 0, 1]
        ])
        self.tac = np.matmul(self.tab, self.tbc) 

    def pose_callback(self, msg: ArucoMarkers):
        for i, num in enumerate(msg.marker_ids):

        # if len(msg.markers_id)
        #     for i in range(2):
                
            x = msg.poses[i].position.x
            y = msg.poses[i].position.y
            z = msg.poses[i].position.z

            rx = msg.poses[i].orientation.x
            ry = msg.poses[i].orientation.y
            rz = msg.poses[i].orientation.z
            rw = msg.poses[i].orientation.w

            in_d = np.array([x, y, z, 1])

            if num == 10:
                self.cube_pos = np.matmul(self.tac, in_d.T)
                x_ = self.cube_pos.item(0,0)
                y_ = self.cube_pos.item(0,1)
                z_ = self.cube_pos.item(0,2)
                self.cube_rot = [rx, ry, rz, rw]
                self.point_3 = [x_, y_, 0.7, 1]
                self.point_4 = [x_, y_, 0.1, 1]
                 
            elif num == 1:
                self.paper_pos = np.matmul(self.tac, in_d.T)
                self.paper_rot = [rx, ry, rz, rw]
                x_ = self.paper_pos.item(0,0)
                y_ = self.paper_pos.item(0,1)
                z_ = self.paper_pos.item(0,2)
                self.point_1 = [x_, y_, 0.7, 1]
                self.point_2 = [x_, y_, 0.1, 1]

    def operation(self):
        while (
            self.point_1 is None or self.point_3 is None
        ) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec= 1 / fr)

        # 2) Move above the box
        self.move_to_point(self.point_3)
        while self.in_action == True:
            rclpy.spin_once(self)

        # 3) Rotate tool
        # self.rotate_tool(self.cube_rot)
        # while self.in_action == True:
        #     rclpy.spin_once(self)

        # 4) Move down to the box
        self.move_to_point(self.point_4)
        while self.in_action == True:
            rclpy.spin_once(self)

        self.change_holding(True)
        while self.in_action == True:
            rclpy.spin_once(self)

        # 5) Move up
        self.move_to_point(self.point_3)
        while self.in_action == True:
            rclpy.spin_once(self)

        # 6) Move above the paper
        self.move_to_point(self.point_1)
        while self.in_action == True:
            rclpy.spin_once(self)

        # 7) Rotate tool
        # self.rotate_tool(self.paper_rot)
        # while self.in_action == True:
        #     rclpy.spin_once(self)

        # 8) Move down to the paper
        self.move_to_point(self.point_2)
        while self.in_action == True:
            rclpy.spin_once(self)

        self.change_holding(False)
        while self.in_action == True:
            rclpy.spin_once(self)

        # 9) Move up
        self.move_to_point(self.point_1)
        while self.in_action == True:
            rclpy.spin_once(self)

        # # 10) Return
        # self.rotate_tool([1, 0, 0, 0])
        # while self.in_action == True:
        #     rclpy.spin_once(self)
    
    def rotate_tool(self, quat):
        [r, p, y] = quat2euler(quat)

        self.tool_rot = y  - self.angle_1
        move_request = PointToPoint.Goal()
        move_request.target_pose = [0.0, 0.0, 0.0, self.tool_rot]
        move_request.motion_type = 1
        future = self.move_client.send_goal_async(move_request)
        future.add_done_callback(self.goal_response_callback)      


    def move_to_point(self, point):
        self.in_action = True
        dx = point[0]
        dy = point[1]
        dz = point[2]
        self.get_logger().info(f"{dx}, {dy}, {dz}")
        move_request = PointToPoint.Goal()
        move_request.target_pose = [1000*dx-42, 1000*dy, 100*dz, 0.0]
        move_request.motion_type = 1
        future = self.move_client.send_goal_async(move_request)
        future.add_done_callback(self.goal_response_callback)

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

    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.in_action = False


def main(args=None):
    rclpy.init(args=args)
    client = FinalBoss()
    client.operation()
    rclpy.spin_once(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()