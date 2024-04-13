import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PointStamped
from numpy import cos, sin 
import sympy as sp
from sympy import symbols

class inverse_kin(Node):
    def __init__(self):
        super().__init__('inverse_kin')
        self.point_subscriber = self.create_subscription(PointStamped, 'clicked_point', self.callback, 10)
        self.angle_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.angle_vector = [1, 1, 1, 1, 1]


    def callback(self, msg):
        self.get_logger().info("Otrzymano wiadomość na temat 'clicked_point'")
        if msg.point is not None:
            x = msg.point.x
            y = msg.point.y
            z = msg.point.z
            self.get_logger().info(f"Otrzymano kliknięty punkt: x={x}, y={y}, z={z}")
        else:
            self.get_logger().info("Otrzymano wiadomość z pustym punktem")

        #self.get_logger().info(x)
        # qx = msg.pose.orientation.x
        # qy = msg.pose.orientation.y
        # qz = msg.pose.orientation.z
        # qw = msg.pose.orientation.w

        # translation = sp.Matrix([
        # [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw), x],
        # [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw), y],
        # [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2), z],
        # [0, 0, 0, 1]
        # ])

        # translation = sp.Matrix([[0, 0, 0, x],
        #                          [0, 0, 0, y],
        #                          [0, 0, 0, z],
        #                          [0, 0, 0, 1]]                               
        #                         )

        # theta1 = symbols('theta1')
        # theta2 = symbols('theta2')
        # theta3 = symbols('theta3')
        # theta4 = symbols('theta4')
        # theta5 = symbols('theta5')


        # m1 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1), 0, 0],
        #                            [sp.sin(theta1), sp.cos(theta1), 0, 0],
        #                            [0, 0, 1, 0.05],
        #                            [0, 0, 0, 1]                                   
        #                            ])
        
        # m2 = sp.Matrix([
        #             [sp.cos(theta2), 0, sp.sin(theta2), 0],
        #             [0, 1, 0, 0],
        #             [-sp.sin(theta2), 0, sp.cos(theta2), 0.088],
        #             [0, 0, 0, 1]
        # ])

        # m3 = sp.Matrix([
        #             [sp.cos(theta3), 0, sp.sin(theta3), 0],
        #             [0, 1, 0, 0],
        #             [-sp.sin(theta3), 0, sp.cos(theta3), 0.135],
        #             [0, 0, 0, 1]
        # ])

        # m4 = sp.Matrix([
        #             [sp.cos(theta4), 0, sp.sin(theta4), 0.147],
        #             [0, 1, 0, 0],
        #             [-sp.sin(theta4), 0, sp.cos(theta4), 0],
        #             [0, 0, 0, 1]
        # ])

        # m5 = sp.Matrix([[sp.cos(theta5), -sp.sin(theta5), 0, 0.03],
        #                            [sp.sin(theta5), sp.cos(theta5), 0, 0],
        #                            [0, 0, 1, -0.024-0.04],
        #                            [0, 0, 0, 1]                                   
        #                            ])

        # m6 = sp.Matrix([
        #             [1, 0, 0, 0],
        #             [0, cos(np.pi), -sin(np.pi), 0],
        #             [0, sin(np.pi), cos(np.pi), 0],
        #             [0, 0, 0, 1]
        # ])

        # result = m1*m2*m3*m4*m5*m6
        # self.get_logger().info("Po result")
        # equation = result - translation
        # solutions = sp.solve(equation, (theta1, theta2, theta3, theta4, theta5))
        # self.get_logger().info("Po solutions.")
        
        # if solutions:
        #     for i in range(5):
        #         self.angle_vector[i] = solutions[i]
        #         self.get_logger().info("Znaleziono rozwiązania:", solutions)
        # else:
        #     self.get_logger().info("Nie znaleziono rozwiązań dla kinematyki odwrotnej.")

        theta1 = 0
        theta2 = 0
        theta3 = 0
        theta4 = 0
        theta5 = 0

        d1 = 0.05
        d2 = 0.088
        d3 = 0.135
        d4 = 0.147
        d5 = 0.064

        theta1 = np.arctan2(y,x)


        pz = z + d5 - d1 - d2
        px = x - 0.03
        py = y

        t3_num = px**2 + pz**2 - d3**2 - d4**2
        t3_den = 2*d3*d4

        theta3 = np.arccos(t3_num/t3_den) - np.pi/2
        theta3_prime = np.arccos(t3_num/t3_den)    


        t2_num = d4*sin(theta3_prime)
        t2_den = np.sqrt(px**2 + pz**2)


        theta2 = np.pi/2 - (np.arcsin(t2_num/t2_den) + np.arctan2(pz, px))

        theta4 = -(theta2 + theta3)
        theta5 = 0.0

        self.get_logger().info(f"angle 1: {theta1}")
        self.get_logger().info(f"angle 2: {theta2}")
        self.get_logger().info(f"angle 3: {theta3}")
        self.get_logger().info(f"angle 4: {theta4}")
        self.get_logger().info(f"angle 5: {theta5}")

        if theta1 < -1.548 or theta1 > 1.548 or theta2 < 0 or theta2 > 1.484 or theta3 < -0.175 or theta3 > 1.571:
            for i in range (5):
                self.angle_vector[i] = 0.0
        else:
            self.angle_vector[0] = theta1
            self.angle_vector[1] = theta2
            self.angle_vector[2] = theta3
            self.angle_vector[3] = theta4
            self.angle_vector[4] = theta5

        #self.get_logger().info(self.angle_vector)

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['rotating_1_join', 'arm_2_joint', 'arm_3_joint', 'tool_pos_joint', 'tool_rot_joint']
        joint_states.position = self.angle_vector
        self.angle_publisher.publish(joint_states)

def main(args=None):
    rclpy.init(args=args)
    client = inverse_kin()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()