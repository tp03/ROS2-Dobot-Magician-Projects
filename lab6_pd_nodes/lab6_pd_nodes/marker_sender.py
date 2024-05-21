import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Point, Quaternion
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Pose

from numpy import sin, cos

cx = 0.2
cy = 0.04
cz = 0.011

px = 0.2
py = 0.0
pz = 0.0005

camera_effector_difference_x = 0.1
camera_effector_difference_z = 0

class MarkerSender(Node):
    def __init__(self):
        super().__init__('marker_sender')
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.marker_publisher = self.create_publisher(PoseArray, 'camera_link', 10)

    def joint_states_callback(self, msg: JointState):
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
        self.publish_markers()

    def publish_markers(self):
        cube_pose = np.array([cx, cy, cz, 1])
        paper_pose = np.array([px, py, pz, 1])

        cp = np.matmul(np.linalg.inv(self.tac), cube_pose.T)
        pp = np.matmul(np.linalg.inv(self.tac), paper_pose.T)

        cube_point = Point(x=cp.item(0,0), y=cp.item(0,1), z=cp.item(0,2))
        paper_point = Point(x=pp.item(0,0), y=pp.item(0,1), z=pp.item(0,2))

        self.cube_marker_pose = Pose(position=cube_point,
                                          orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))
        
        self.paper_marker_pose = Pose(position=paper_point,
                                          orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))

        marker_pose_array = PoseArray()
        marker_pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_pose_array.header.frame_id = 'camera_link'
        marker_pose_array.poses = [self.cube_marker_pose, self.paper_marker_pose]

        self.marker_publisher.publish(marker_pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

