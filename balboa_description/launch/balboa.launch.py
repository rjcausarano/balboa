# @author Rodrigo Causarano (rjcausarano@gmail.com)

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Directory
    pkg_balboa_description = get_package_share_directory('balboa_description')
    # Path
    balboa_xacro_file = PathJoinSubstitution(
        [pkg_balboa_description, 'urdf', 'balboa.urdf.xacro'])

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             ''],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dock_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro', ' ', balboa_xacro_file])},
        ],
    )

    pitch_controller = Node(
        package='balboa_controllers',
        executable='pitch_controller_node',
        name='pitch_controller_node',
        output='screen',
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        arguments=['-entity',
                   'balboa',
                   '-topic',
                   'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-Y', '0.0'],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(state_publisher)
    ld.add_action(spawn_model)
    ld.add_action(pitch_controller)

    return ld
