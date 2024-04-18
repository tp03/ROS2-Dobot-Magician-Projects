import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

class Move_To_Point(Node):
    def __init__(self):
        super().__init__('Move_to_Point')
        self.point_subscriber = self.create_subscription(PointStamped, 'clicked_point', self.callback, 10)
        self.move_client = ActionClient(self, PointToPoint, '/PTP_action')

    def callback(self ,msg):
        self.get_logger().info("Otrzymano wiadomość na temat 'clicked_point'")
        if msg.point is not None:
            x = msg.point.x
            y = msg.point.y
            z = msg.point.z
            self.get_logger().info(f"Otrzymano kliknięty punkt: x={x}, y={y}, z={z}")
        else:
            self.get_logger().info("Otrzymano wiadomość z pustym punktem")


        final_x = 100*x
        final_y = 100*y
        final_z = 100*z

        move_request = PointToPoint.Goal()
        move_request.target_pose = [final_x, final_y, final_z, 0.0]
        move_request.motion_type = 1

        future = self.move_client.send_goal_async(move_request)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.in_action = False

def main(args=None):
    rclpy.init(args=args)
    client =  Move_To_Point()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()