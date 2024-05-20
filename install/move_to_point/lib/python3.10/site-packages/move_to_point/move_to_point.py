import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

class MoveToPoint(Node):
    def __init__(self):
        super().__init__('Move_to_Point')
        self.point_subscriber = self.create_subscription(PointStamped, 'clicked_point', self.callback, 10)
        self.move_client = ActionClient(self, PointToPoint, 'PTP_action')

    def callback(self ,msg):
        self.get_logger().info("Otrzymano wiadomość na temat 'clicked_point'")
        if msg.point is not None:
            self.x = msg.point.x
            self.y = msg.point.y
            self.z = msg.point.z
            self.get_logger().info(f"Otrzymano kliknięty punkt: x={self.x}, y={self.y}, z={self.z}")
        else:
            self.get_logger().info("Otrzymano wiadomość z pustym punktem")


        angle = np.arctan2(self.x, self.y)
        
        final_x = 1000*(self.x - 0.025*np.sin(angle))
        final_y = 1000*(self.y - 0.025*np.cos(angle))
        final_z = 1000*(self.z - 0.06)


        move_request = PointToPoint.Goal()
        move_request.target_pose = [final_x, final_y, final_z, 0.0]
        move_request.motion_type = 1

        self.move_client.wait_for_server()
        self.send_goal_future = self.move_client.send_goal_async(move_request, feedback_callback = self.feedback)
        self.send_goal_future.add_done_callback(self.check)

    def feedback(self, future):
        self.get_logger().info("Moving...")
    
    def check(self, future):
        goal = future.result()
        if goal.accepted:
            self.get_logger().info("OK")


                
def main(args=None):
    rclpy.init(args=args)
    client =  MoveToPoint()
    rclpy.spin(client)
    rclpy.destroy_node(client)


if __name__ == '__main__':
    main()
