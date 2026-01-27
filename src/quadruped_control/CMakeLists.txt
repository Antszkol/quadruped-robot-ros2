cmake_minimum_required(VERSION 3.8)
project(quadruped_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. Szukaj nagłówków w folderze include
include_directories(include)

# 2. Buduj bibliotekę matematyczną (IK + Trajectory)
add_library(quadruped_lib
  src/IK_solver.cpp
  src/Trajectory_Generator.cpp
)
ament_target_dependencies(quadruped_lib rclcpp geometry_msgs std_msgs sensor_msgs)

# 3. Buduj główny węzeł (Control Node)
# UWAGA: Tu musi być nazwa Twojego pliku .cpp (widziałem na ls, że masz quadruped_control.cpp)
add_executable(control_node src/quadruped_control.cpp)

# Łączymy węzeł z biblioteką
target_link_libraries(control_node quadruped_lib)
ament_target_dependencies(control_node rclcpp geometry_msgs std_msgs sensor_msgs)

# 4. INSTALACJA (To jest to, czego brakowało lub było błędne!)
install(TARGETS
  quadruped_lib
  control_node   # <--- To kopiuje plik wykonywalny do folderu install/lib/...
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()