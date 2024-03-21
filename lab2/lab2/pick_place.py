import rclpy
import rclpy.node
from time import sleep

class Pick_place(rclpy.node.Node):
    def __init__(self):
        super().__init__('pick_place')

        self.base_x = 0
        self.base_y = 0
        self.base_z = 0

        self.cube_x = 0
        self.cube_y = 0
        self.cube_z = 0

        self.cube_height = 30

    def jump(self):
        sleep(1)
        self.in_action = True
        self.in_action = False
    def change_holding(self):
        sleep(1)
        self.in_action = True
        self.in_action = False
    
    #possible to use

    def set_base_position(self, x, y, z):
        self.base_x = x
        self.base_y = y
        self.base_z = z

    def set_first_cube_position(self, x, y, z):
        self.cube_x = x
        self.cube_y = y
        self.cube_y = z

def main():
    rclpy.init()
    client = Pick_place()
    client.set_base_position(30, 30, 30)
    client.set_first_cube_position(60, 60, 30)
    rclpy.shutdown()

if __name__ == '__main__':
    main()