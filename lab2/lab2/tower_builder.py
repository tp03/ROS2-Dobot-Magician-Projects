import rclpy
from rclpy import Node

class Tower_builder(Node):
    def __init__(self):
        super().__init__('tower_builder')
        self.declare_parameter(name='tower_size')
    def jump(self):
        pass
    def change_holding(self):
        pass

def main():
    rclpy.init()
    client = Tower_builder()
    rclpy.shutdown()

if __name__ == '__main__':
    main()