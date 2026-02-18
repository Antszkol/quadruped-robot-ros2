cmake_minimum_required(VERSION 3.8)
project(admittance_reg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(adm_reg_node src/admittance_reg_test.cpp)

ament_target_dependencies(adm_reg_node rclcpp std_msgs)

install(TARGETS adm_reg_node DESTINATION lib/admittance_reg)

ament_package()
