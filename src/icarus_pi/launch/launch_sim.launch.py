import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='icarus_pi'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.wheel.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'icarus_pi'],
                        output='screen')

    controller_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",   #on humble it is no longer spawner.py
        arguments=["mech_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",   #on humble it is no longer spawner.py
        arguments=["joint_broad"],
)
    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        #condition = launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
)

    joint_state_publisher_gui_node =Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name = 'joint_state_publisher_gui',
        condition = launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        joint_state_publisher_node,
        gazebo,
        spawn_entity,
        #joystick,
        #controller_drive_spawner,
        joint_broad_spawner
    ])
