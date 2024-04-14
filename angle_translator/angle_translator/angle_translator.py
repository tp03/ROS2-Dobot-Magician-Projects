import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class angle_translator(Node):
    def __init__(self):
        super().__init__('angle_translator')
        self.dobot_subscription = self.create_subscription(JointState, 'dobot_joint_states', self.listener_callback, 10)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.theta_vector = []
        self.answer_vector = [1, 1, 1, 1, 1]
        


    def listener_callback(self, msg):
        self.theta_vector = msg.position
        self.angle_calculator()
        self.publish_joint_states()



    def angle_calculator(self):
        theta_1 = self.theta_vector[0]
        theta_2 = self.theta_vector[1]
        theta_3 = self.theta_vector[2]
        theta_5 = self.theta_vector[3] + np.pi/2

        self.answer_vector[0] = theta_1
        self.answer_vector[1] = theta_2
        self.answer_vector[2] = theta_3
        self.answer_vector[4] = theta_5

        theta_4 = -(theta_2+theta_3)

        self.answer_vector[3] = theta_4
   

    def publish_joint_states(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['rotating_1_join', 'arm_2_joint', 'arm_3_joint', 'tool_pos_joint', 'tool_rot_joint']
        joint_states.position = self.answer_vector
        self.joint_publisher.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)
    client = angle_translator()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


