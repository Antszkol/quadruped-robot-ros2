import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Ustawienie ścieżek ---
    pkg_name = 'dogbot_description' 
    urdf_file_name = 'dogbot.urdf' 
    
    pkg_path = get_package_share_directory(pkg_name)
    
    # Poprawna ścieżka do pliku URDF
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file_name)

    # Wczytanie zawartości pliku URDF do zmiennej
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # --- 2. Uruchomienie Gazebo ---
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf -v 4'}.items()
    )

    # --- 3. Węzeł Robot State Publisher ---
    # Publikuje URDF i /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # --- 4. Węzeł Spawnera Robota ---
    # Ładuje model do Gazebo z tematu /robot_description
    spawn_entity_node = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'dogbot',
                   '-z', '0.2'], # Spawnuj 20cm nad ziemią
        output='screen'
    )

    # --- 5. Węzły Spawnerów Kontrolerów (KRYTYCZNE!) ---
    
    # Uruchamia "sprawozdawcę"
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Uruchamia "siłacza" (trzymającego stawy)
    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # --- 6. Zwrócenie listy wszystkich węzłów do uruchomienia ---
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller
    ])