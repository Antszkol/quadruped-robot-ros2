cmake_minimum_required(VERSION 3.8)
project(dogbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź wszystkie niezbędne pakiety
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(controller_manager REQUIRED)

# === PEŁNA LISTA INSTALACJI ===
# Instalujemy *wszystkie* Twoje foldery z zasobami.

install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Dodane brakujące foldery
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY
  sdf/
  DESTINATION share/${PROJECT_NAME}/sdf
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
