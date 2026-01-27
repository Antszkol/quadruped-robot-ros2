cmake_minimum_required(VERSION 3.8)
project(esp32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- KLUCZOWA ZMIANA 1: Wskazanie folderu z nagłówkami ---
include_directories(include) 

# --- Reszta konfiguracji ---
add_executable(udp_bridge_receiver_node src/udp_bridge_receiver.cpp)
ament_target_dependencies(udp_bridge_receiver_node rclcpp sensor_msgs)

add_executable(udp_bridge_sender_node src/udp_bridge_sender.cpp)
ament_target_dependencies(udp_bridge_sender_node rclcpp sensor_msgs)

# Instalacja nagłówków (ważne, by inne pakiety mogły z nich korzystać w przyszłości)
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  udp_bridge_receiver_node
  udp_bridge_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()