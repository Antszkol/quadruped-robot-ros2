cmake_minimum_required(VERSION 3.8)
project(quadruped_servotranslator_cpp)

# Ustawienie standardu C++ dla bezpieczeństwa
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdowanie paczek
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Deklaracja binarki
add_executable(translator_node src/QuadrupedTranslatorNode.cpp)

# Ścieżki do plików nagłówkowych
target_include_directories(translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Łączenie z zależnościami
ament_target_dependencies(translator_node 
  rclcpp 
  std_msgs 
  geometry_msgs
)

# Instalacja binarki
install(TARGETS
  translator_node
  DESTINATION lib/${PROJECT_NAME}
)

# DODAJ TO: Instalacja skryptów launch (bardzo przydatne!)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()