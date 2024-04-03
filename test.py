import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatesConverter(Node):
    def __init__(self):
        super().__init__('joint_states_converter_node')
        self.subscription = self.create_subscription(JointState, '/dobot_joint_states', self.listener_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        

    def publish_joint_states(self, joint_positions):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ...
        joint_states.position = ....
        self.publisher.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)



if __name__ == '__main__':
    main()

