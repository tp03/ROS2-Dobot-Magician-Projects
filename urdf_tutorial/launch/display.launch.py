from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdf_tutorial_path = FindPackageShare('urdf_tutorial')
    default_model_path = PathJoinSubstitution(['urdf', '01-myfirst.urdf'])
    default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'urdf.rviz'])

    # These parameters are maintained for backwards compatibility
    # gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui')
    # ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'urdf_tutorial',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),}.items()
            # 'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    # ld.add_action(Node(
    #         package='angle_translator',
    #         executable='angle_translator',
    #         name='angle_node',
    #     ),)
    
    ld.add_action(Node(
            package='forward_kin',
            executable='forward_kin',
            name='forward_node',
        ),)
    
    ld.add_action(Node(
            package='inverse_kin',
            executable='inverse_kin',
            name='inverse_node',
        ),)
    
    ld.add_action(Node(
            package='lab6_pd_nodes',
            executable='robot_pilot',
            name='pilot_node',

    ),)

    ld.add_action(Node(
            package='lab6_pd_nodes',
            executable='marker_publisher',
            name='marker_node',

    ),)

    # ld.add_action(Node(
    #         package='move_to_point',
    #         executable='move_to_point',
    #         name='move_node',
    #     ),)


    return ld

