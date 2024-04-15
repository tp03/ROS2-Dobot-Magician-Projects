import rclpy
from rclpy.node import Node
import numpy as np
from numpy import sin, cos 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R


class ForwardKin(Node):

    def __init__(self):
        super().__init__("ForwardKin")
        
        self.pose_publisher = self.create_publisher(PoseStamped, 'end_effector_pose', 10)
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.theta_vector = []

    def calculate_position(self):
        joint_count = len(self.theta_vector)

        matrixes = []

        matrixes.append(np.matrix([[cos(self.theta_vector[0]), -sin(self.theta_vector[0]), 0, 0],
                                   [sin(self.theta_vector[0]), cos(self.theta_vector[0]), 0, 0],
                                   [0, 0, 1, 0.05],
                                   [0, 0, 0, 1]                                   
                                   ]))
        
        matrixes.append(np.matrix([
                    [cos(self.theta_vector[1]), 0, sin(self.theta_vector[1]), 0],
                    [0, 1, 0, 0],
                    [-sin(self.theta_vector[1]), 0, cos(self.theta_vector[1]), 0.088],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(self.theta_vector[2]), 0, sin(self.theta_vector[2]), 0],
                    [0, 1, 0, 0],
                    [-sin(self.theta_vector[2]), 0, cos(self.theta_vector[2]), 0.135],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(self.theta_vector[3]), 0, sin(self.theta_vector[3]), 0.147],
                    [0, 1, 0, 0],
                    [-sin(self.theta_vector[3]), 0, cos(self.theta_vector[3]), 0],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([[cos(self.theta_vector[4]), -sin(self.theta_vector[4]), 0, 0.03],
                                   [sin(self.theta_vector[4]), cos(self.theta_vector[4]), 0, 0],
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
        
        result = matrixes[5]
        return result
        
    def joint_states_callback(self, msg):
        self.theta_vector = msg.position
        grip = self.calculate_position()
        position = grip[:3, 3]
        orientation = grip[:3, :3]
        quaternion = R.from_matrix(orientation).as_quat()
        end_pose = PoseStamped()
        end_pose.header.stamp = self.get_clock().now().to_msg()
        end_pose.header.frame_id = 'base_link'
        end_pose.pose.position.x = position.item(0, 0)
        end_pose.pose.position.y = position.item(1,0)
        end_pose.pose.position.z = position.item(2,0)
        end_pose.pose.orientation.x = quaternion[0]
        end_pose.pose.orientation.y = quaternion[1]
        end_pose.pose.orientation.z = quaternion[2]
        end_pose.pose.orientation.w = quaternion[3]
        self.pose_publisher.publish(end_pose)

def main(args=None):

    rclpy.init(args=args)
    client = ForwardKin()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
    main()

