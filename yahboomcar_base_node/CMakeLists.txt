# yahboomcar_base_node/CMakeLists.txt
# Pacote C++ de odometria — gera 3 executáveis independentes (X3, x1, R2).
# Nota: talker.cpp existe no src/ mas NÃO é compilado — é boilerplate de exemplo.

cmake_minimum_required(VERSION 3.5)
project(yahboomcar_base_node)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)           # quaternion e transformações matemáticas
find_package(tf2_ros REQUIRED)       # TransformBroadcaster (TF odom→base_footprint)
find_package(nav_msgs REQUIRED)      # nav_msgs/Odometry
find_package(geometry_msgs REQUIRED) # geometry_msgs/Twist (vel_raw)
find_package(rclcpp REQUIRED)
# turtlesim listado como dependência no package.xml mas não usado no código
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 3 executáveis independentes — um por variante de robô
add_executable(base_node_X3 src/base_node_X3.cpp)  # X3 mecanum omnidirecional
add_executable(base_node_x1 src/base_node_x1.cpp)  # x1 omnidirecional (menor)
add_executable(base_node_R2 src/base_node_R2.cpp)  # R2 Ackermann (steering)

ament_target_dependencies(base_node_X3 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)
ament_target_dependencies(base_node_x1 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)
ament_target_dependencies(base_node_R2 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)

install(TARGETS
  base_node_X3
  base_node_x1
  base_node_R2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
