import rclpy
from rclpy.node import Node
import numpy as np
from numpy import sin, cos 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R


class ForwardKin(Node):

    def __init__(self):
        super().__init__("ForwardKin")
        
        self.pose_publisher = self.create_publisher(PoseStamped, 'end_effector_pose', 10)
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        self.link1_length = 0.138
        self.link2_length = 0.135
        self.link3_length = 0.147
        self.link4_length = 0.05

        self.theta_vector = []

    def make_matrix(self, d_i, a_i, alpha_i, theta_i):
        a = np.matrix([[cos(theta_i), -sin(theta_i), 0, a_i],
                      [sin(theta_i)*cos(alpha_i), cos(alpha_i)*cos(theta_i), -sin(alpha_i), -d_i*sin(alpha_i)],
                      [sin(theta_i)*sin(alpha_i), cos(theta_i)*sin(alpha_i), cos(alpha_i), d_i*cos(alpha_i)],
                      [0, 0, 0, 1]])
        return a

    def calculate_position(self):
        joint_count = len(self.theta_vector)

        #self.theta_vector[4] = -self.theta_vector[3]
        #self.theta_vector[2] = self.theta_vector[2]-self.theta_vector[1]
        #self.theta_vector[3] = -(self.theta_vector[1]+self.theta_vector[2])

        self.theta_vector[1] = self.theta_vector[1]-np.pi/2
        self.theta_vector[2] = self.theta_vector[2]+np.pi/2

        d = [0.05, 0.088, 0, 0, self.link4_length]
        a = [0, 0, self.link2_length, self.link3_length, 0]
        alpha = [0, -np.pi/2, 0, 0, -np.pi/2]

        matrixes = []

        for i in range(joint_count):
            matrixes.append(self.make_matrix(d[i], a[i], alpha[i], self.theta_vector[i]))

        for i in range(joint_count-1):
            R = np.matmul(matrixes[i], matrixes[i+1])
            matrixes[i+1] = R
        
        result = matrixes[4]
        print(matrixes[4])
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
        self.get_logger().info('Published end effector pose:\n{}'.format(end_pose))

def main(args=None):

    rclpy.init(args=args)
    client = ForwardKin()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
    main()




    
