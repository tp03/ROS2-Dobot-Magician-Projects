from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='dobot_bringup',
        #     namespace='robot',
        #     executable='DobotClient_PTP',
        #     name='my_node'
        # ),
        Node(
            package='lab2',
            executable='pick_place',
            namespace='robot',
            name='my_node',
            # parameters=[{
            # 	'tower_size': LaunchConfiguration('tower_size'),
            # 	},
            # 	]
        ),
])