import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # parse Launch description
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    package_name = "hubert"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py"
            )
        ]),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "gazebo_params.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ]),

        # on launch there is an error displayed: "Error document empty." But the parameters are actually properly parsed.
        launch_arguments={
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "hubert"],
        output="screen")


    launch_description = LaunchDescription([
            use_sim_time_launch_arg,
            rsp,
            gazebo,
            spawn_entity
        ])
    
    return launch_description
     


