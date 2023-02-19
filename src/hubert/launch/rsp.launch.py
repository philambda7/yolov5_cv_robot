import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # parse Launch description
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    use_ros2_control_launch_arg =  DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control if true'
    )


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('hubert'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # rsp
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        use_sim_time_launch_arg,
        use_ros2_control_launch_arg,
        node_robot_state_publisher
    ])