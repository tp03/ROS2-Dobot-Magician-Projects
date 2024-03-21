from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='ROS2',
            executable='my_turtle_node',
            namespace='turtlesim1',
            name='my_node',
            parameters=[{
            	'turtle2name': LaunchConfiguration('turtle2name'),
            	},
            	{
            	'turtle23name': LaunchConfiguration('turtle3name'),
            }]

        ),
    ])