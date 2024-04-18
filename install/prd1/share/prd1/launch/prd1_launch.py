from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'turtlesim',
            namespace= 'turtleeeees',
            executable= 'turtlesim_node',
        ),
        Node(
            package = 'prd1',
            namespace= 'turtleeeees',
            executable='skz',
            parameters=[{
            	"turtle2": LaunchConfiguration('turtle2'),
            	},
            	{
            	"turtle3": LaunchConfiguration('turtle3'),
            }]
        )       
    ])
