import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from ros2_aruco_interfaces.msg import ArucoMarkers
import numpy as np
from numpy import sin, cos

camera_effector_difference_x = 0.04
camera_effector_difference_y = 0.07
camera_effector_difference_z = 0.06

class MarkerBroker(Node):
    def __init__(self):
        super().__init__('marker_broker')
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.m_subscriber = self.create_subscription(ArucoMarkers, 'aruco_markers', self.pose_callback, 10)
        self.flag_subscriber = self.create_subscription(Bool , 'important_flag', self.flag_callback, 10)
        self.m_publisher = self.create_publisher(Marker, 'vizualization_marker', 10)
        self.camera_position = []
        self.publish = True
        self.paper_pos = None
        self.cube_pos = None

    def flag_callback(self, msg: Bool):
        if msg.data:
            self.publish = False
        else:
            self.publish = True

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
                    self.cube_rot = [rx, ry, rz, rw]
                elif num == 1:
                    self.paper_pos = np.matmul(self.tac, in_d.T)
                    self.paper_rot = [rx, ry, rz, rw]
                
        if (self.cube_pos is not None) | (self.paper_pos is not None):   
            self.publish_marker()
            self.paper_pos = None
            self.cube_pos = None

    def publish_marker(self):
        # new_msg = Bool()
        # new_msg.data = False
        # self.finish_publisher.publish(new_msg)
        marker1 = Marker()
        marker2 = Marker()
        
        
        if self.cube_pos is not None:
            marker1.header.frame_id = "base_link"
            marker1.header.stamp = self.get_clock().now().to_msg()
            
            marker1.id = 10
            marker1.type = Marker.CUBE
            marker1.action = Marker.ADD
            marker1.pose.position.x = self.cube_pos.item(0,0)
            marker1.pose.position.y = self.cube_pos.item(0,1)
            marker1.pose.position.z = self.cube_pos.item(0,2)
            marker1.pose.orientation.x = -self.cube_rot[1]
            marker1.pose.orientation.y = -self.cube_rot[0]
            marker1.pose.orientation.z = -self.cube_rot[2]
            marker1.pose.orientation.w = self.cube_rot[3]

        
            marker1.scale.x = 0.02
            marker1.scale.y = 0.02
            marker1.scale.z = 0.02
            
            marker1.color.r = 0.0
            marker1.color.g = 1.0
            marker1.color.b = 0.0
            marker1.color.a = 1.0
        
            self.m_publisher.publish(marker1)

       
        if self.paper_pos is not None:
            marker2.header.frame_id = "base_link"
            marker2.header.stamp = self.get_clock().now().to_msg()
            
            marker2.id = 1
            marker2.type = Marker.CUBE
            marker2.action = Marker.ADD
            marker2.pose.position.x = self.paper_pos.item(0,0)
            marker2.pose.position.y = self.paper_pos.item(0,1)
            marker2.pose.position.z = self.paper_pos.item(0,2)
            marker2.pose.orientation.x = -self.paper_rot[1]
            marker2.pose.orientation.y = -self.paper_rot[0]
            marker2.pose.orientation.z = -self.paper_rot[2]
            marker2.pose.orientation.w = self.paper_rot[3]
            
            marker2.scale.x = 0.05
            marker2.scale.y = 0.1
            marker2.scale.z = 0.001
            
            marker2.color.r = 0.0
            marker2.color.g = 0.0
            marker2.color.b = 1.0
            marker2.color.a = 1.0

            self.m_publisher.publish(marker2)

def main(args=None):
    rclpy.init(args=args)
    marker_broker = MarkerBroker()
    rclpy.spin(marker_broker)

    marker_broker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()