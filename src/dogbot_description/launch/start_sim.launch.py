import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'dogbot_description'
    robot_file = 'dogbot.urdf'
    pkg_path = get_package_share_directory(pkg_name)
    robot_desc_path = os.path.join(pkg_path, robot_file)

    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            os.path.join(pkg_path, 'dogbot.world')
        ],


        output='screen'
    )

    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-topic', 'robot_description',
            '-name', 'dogbot'
        ],

        output='screen'
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    spawn_joint_trajectory_controller = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller,
    ])
