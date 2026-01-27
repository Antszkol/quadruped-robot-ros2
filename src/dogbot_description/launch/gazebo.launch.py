# Author: Antoni Biskupski
# Date: October 29th, 2025 (Updated for Gazebo Harmonic Compatibility)
# GLOWNY PLIK DO LAUNCHOWANIA SYMULACJI Z ROS2_CONTROL

import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
    # 1. STAŁE I ŚCIEŻKI
    package_name = 'dogbot_description'
    robot_name_in_model = 'dogbot'
    
    # KLUCZOWE ŚCIEŻKI
    # Wymagany plik kontrolera dla joint_state_broadcaster i GTC
    ros2_controllers_config_path = 'config/dogbot_controllers.yaml' 
    bridge_config_file_path = 'config/dogbot_bridge.yaml' 
    rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'
    urdf_file_path = 'urdf/dogbot.urdf.xacro'
    world_file_path = 'worlds/empty.sdf'
        
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.1' # Podniesiony z, żeby nie wpadał w podłoże
    spawn_yaw_val = '0.00'
 
    # 2. DEFINICJE ŚCIEŻEK (KONSTRUKCJA)
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    controllers_config_path = os.path.join(pkg_share, ros2_controllers_config_path)
    print(f"DEBUG: Controllers YAML path: {controllers_config_path}")
 
    # 3. ZMIENNE KONFIGURACYJNE (ARGUMENTY)
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')
    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 4. DEKLARACJA ARGUMENTÓW (Bez zmian, są poprawne)
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path,
        description='Absolute path to robot xacro file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    # 5. DEFINICJE WĘZŁÓW
    
    # Węzeł 5.1: Robot State Publisher (TF)
    # Wczytuje URDF i publikuje transformacje TF, absolutnie kluczowe!
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_model]), 
                     'use_sim_time': use_sim_time}])
    # Węzeł 5.9: Wydawca Statycznej Transformacji (ROZWIĄZANIE PROBLEMU RViz)
    # Łączy ramę świata 'map' z ramą robota 'base_link'
    start_static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link_broadcaster',
        output='screen',
        # Argumenty: x y z yaw pitch roll parent_frame child_frame
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # Węzeł 5.2: Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    
    # Węzeł 5.3: START GAZEBO HARMONIC (IGNITION) SIMULATOR
    start_gazebo_sim_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=IfCondition(use_simulator)
    )
    
    '''
    # Węzeł 5.4: ROS-GAZEBO PARAMETER BRIDGE (Zsynchronizowany z /clock i kontrolerami)
    start_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='dogbot_parameter_bridge',
        output='screen',
        # Pamiętaj o uzupełnieniu dogbot_bridge.yaml!
        arguments=[os.path.join(pkg_share, bridge_config_file_path)],
        condition=IfCondition(use_simulator)
    )
    '''

    start_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    # Uruchom tylko most zegara, który jest kluczowy
    arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    output='screen',
    condition=IfCondition(use_simulator)
)

    # Węzeł 5.5: SPÓŹNIONE SPAWNOWANIE ROBOTA (Musi być po uruchomieniu Gazebo i Bridge)
    spawn_entity_cmd = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-entity', robot_name_in_model, 
                    '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')
    
    # Węzeł 5.6: Załadowanie Controller Managera
    # Węzeł ten wymaga pliku dogbot_ros2_control.yaml
    load_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config_path],
        output='log',
        condition=IfCondition(use_simulator)
    )

    # Węzeł 5.7: SPRAWNER JOINT STATE BROADCASTER
    # Spawner odpowiedzialny za publikację /joint_states z Gazebo do ROS
    start_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_simulator)
    )
    
    # Węzeł 5.8: SPRAWNER KONTROLERA GTC (Quadruped Joint Trajectory Controller)
    start_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["quadruped_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_simulator)
    )

    # 6. TWORZENIE LAUNCH DESCRIPTION I KOLEJNOŚĆ AKCJI
    ld = LaunchDescription()

    # Deklaracje
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    
    # 6.1 Uruchomienie Bazowe (Gazebo, Bridge, RSP, RViz)
    ld.add_action(start_gazebo_sim_cmd) 
    ld.add_action(start_bridge_cmd)
    ld.add_action(start_robot_state_publisher_cmd) 
    ld.add_action(start_rviz_cmd) 
    # Usunięto start_joint_state_publisher_cmd, by uniknąć kolizji kontrolerów.
    
    ld.add_action(start_static_transform_publisher_cmd)

    # 6.2 Czasowe Uruchamianie (Sekwencja Inżynierska)
    
    # Krok 1: Spawn robota po 2.5s (dajemy Gazebo czas na start)
    ld.add_action(
        TimerAction(
            period=2.5, 
            actions=[spawn_entity_cmd], 
        )
    )
    
    # Krok 2: Załaduj Controller Manager po 3.5s (musi być po spawn)
    ld.add_action(
        TimerAction(
            period=7.0, 
            actions=[load_controller_manager], 
        )
    )
    
    # Krok 3: Spawner Joint State Broadcaster po 4.5s (Broadcaster jest wymagany)
    ld.add_action(TimerAction(
        period=4.5, 
        actions=[start_joint_broadcaster],
    ))
    
    # Krok 4: Spawner GTC po 5.5s (Ostatni, musi mieć pewność, że wszystko wcześniej działa)
    ld.add_action(TimerAction(
        period=5.5, 
        actions=[start_controller_spawner],
    ))
    
    return ld