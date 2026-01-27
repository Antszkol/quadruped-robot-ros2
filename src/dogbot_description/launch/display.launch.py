# W pliku: ~/ros2_ws/src/dogbot_description/launch/display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'dogbot_description' 
    file_name = 'dogbot.urdf' # Używamy czystego URDF
    
    pkg_path = get_package_share_directory(pkg_name)
    robot_desc_path = os.path.join(pkg_path, 'urdf', file_name)

    # 1. Wczytanie zawartości URDF/XACRO (KLUCZOWY KROK)
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. robot_state_publisher (używa parametru robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # KLUCZOWE: Upewnij się, że ten parametr jest poprawny
        parameters=[{'robot_description': robot_desc}], 
    )

    # ... (pozostałe węzły bez zmian) ...
    
    # 3. joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])