import rclpy
from rclpy import Node

class Pick_place(Node):
    def __init__(self):
        super().__init__('pick_place')
    def jump(self):
        pass
    def change_holding(self):
        pass

def main():
    rclpy.init()
    client = Pick_place()
    rclpy.shutdown()

if __name__ == '__main__':
    main()