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