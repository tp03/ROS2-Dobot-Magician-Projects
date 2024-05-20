import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
from numpy import sin, cos

#from robot_msgs.action import VisPointToPoint
camera_effector_difference_x = 0.1
camera_effector_difference_z = 0

class RobotPilot(Node):
    def __init__(self):
        super().__init__('robot_pilot')
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        # self.publisher = self.create_publisher(PointStamped, 'target_point', 10)
        self.subscription = self.create_subscription(PoseArray, 'camera_link', self.point_translation, 10)

    def joint_states_callback(self ,msg: JointState):
        theta_vector = msg.position
        joint_count = len(theta_vector)

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
            [1, 0, 0, camera_effector_difference_x],
            [0, 1, 0, 0],
            [0, 0, 1, -camera_effector_difference_z],
            [0, 0, 0, 1]
        ])
        self.tac = np.matmul(self.tab, self.tbc)

    def point_translation(self, msg: PoseArray):
        for i in range(4):
            
            x_ = msg.poses[i].position.x
            y_ = msg.poses[i].position.y
            z_ = msg.poses[i].position.z

            in_d = np.array([x_, y_, z_, 1])

            if i == 0 or i == 2:
                self.cube_pos = np.matmul(self.tac, in_d.T)
                x = self.cube_pos.item(0, 0)
                y = self.cube_pos.item(0, 1)
                z = self.cube_pos.item(0, 2)
            else:
                self.paper_pos = np.matmul(self.tac, in_d.T)
                x = self.paper_pos.item(0, 0)
                y = self.paper_pos.item(0, 1)
                z = self.paper_pos.item(0, 2)
            
            if i == 2 or i == 3:
                z = z + 0.1

            if i == 0 or i == 1:
                z = z + 0.011

            theta1 = 0
            theta2 = 0
            theta3 = 0
            theta4 = 0
            theta5 = 0

            d1 = 0.05
            d2 = 0.088
            d3 = 0.135
            d4 = 0.147
            d5 = 0.069

            theta1 = np.arctan2(y,x)


            pz = z + d5 - d1 - d2
            px = np.sqrt(x**2 + y**2) - 0.03
            alpha = np.arctan2(pz, px)
            m = np.sqrt(px**2 + pz**2)

            t3_num = m**2 - d3**2 - d4**2
            t3_den = 2*d3*d4


            theta3 = np.arccos(t3_num/t3_den) - np.pi/2 

            beta = np.arccos((d3**2 + m**2 - d4**2)/(2*d3*m))
            
            theta2 = np.pi/2 - (alpha + beta)

            theta4 = -(theta2 + theta3)
            theta5 = 0.0

            if i == 2:
                self.first_pos = [theta1, theta2, theta3, theta4, theta5]
            elif i == 0:
                self.second_pos = [theta1, theta2, theta3, theta4, theta5]
            elif i == 3:
                self.third_pos = [theta1, theta2, theta3, theta4, theta5]
            else:
                self.fourth_pos = [theta1, theta2, theta3, theta4, theta5]                

    def move_to_point(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
