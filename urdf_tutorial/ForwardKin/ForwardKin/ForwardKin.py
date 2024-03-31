import rclpy
from rclpy.node import Node
import numpy as np
from numpy import sin, cos 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class ForwardKin(Node):

    def __init__(self):
        super().__init__("ForwardKin")
        
        self.pose_publisher = self.create_publisher(PoseStamped, "end_effector_pose", 10)
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        self.link2_length = 0.135
        self.link3_length = 0.147
        self.link4_length = 0.05
        self.link5_length = 0.07

    def make_matrix(self, d_i, a_i, alpha_i, theta_i):
        a = np.matrix([[cos(theta_i), -sin(theta_i),0 , a_i],
                      [sin(theta_i)*cos(alpha_i), cos(alpha_i)*cos(theta_i), -sin(alpha_i), -d_i*sin(alpha_i)],
                      [sin(theta_i)*sin(alpha_i), cos(theta_i)*sin(alpha_i), cos(alpha_i), d_i*cos(alpha_i)],
                      [0, 0, 0, 1]])
        return a

    def calculate_position(self, theha_vector):
        joint_count = len(theha_vector)

        d = [0, 0, 0, 0, self.link4_length]
        a = [0, 0, self.link2_length, self.link3_length, 0]
        alpha = [0, np.pi/2, 0, 0, -np.pi/2]

        matrixes = []

        for i in range(joint_count):
            matrixes.append(self.make_matrix(d[i], a[i], alpha[i], theha_vector[i]))

        for i in range(joint_count-1):
            R = np.matmul(matrixes[i], matrixes[i+1])
            matrixes[i+1] = R
        
        result = matrixes[4]

        message = [result.item(0, 3), result.item(1, 3), result.item(2, 3)]
        return message
        #print(matrixes[4])

    def joint_states_callback(self, msg):
        #msg = self.calculate_position([np.pi/3, np.pi/3, np.pi/3, 0, 0])
        thetas = msg.position
        self.get_logger().info('connected')
        end_pose = PoseStamped()
        end_pose.header.stamp = self.get_clock().now().to_msg()
        end_pose.header.frame_id = "base_link"
        end_pose.pose.position.x = thetas[0]
        end_pose.pose.position.x = thetas[1]
        end_pose.pose.position.x = thetas[2]
        self.pose_publisher.publish(end_pose)
        self.pose_publisher.publish("hello")

def main(args=None):

    rclpy.init(args=args)
    client = ForwardKin()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
    main()




    
