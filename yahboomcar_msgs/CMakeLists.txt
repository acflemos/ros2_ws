# yahboomcar_msgs/CMakeLists.txt
# Pacote de mensagens customizadas — não gera executáveis, só tipos ROS2.
# O rosidl_generate_interfaces() gera automaticamente código Python e C++
# a partir dos arquivos .msg, tornando os tipos disponíveis via:
#   from yahboomcar_msgs.msg import Target, TargetArray, Position, PointArray, ImageMsg

cmake_minimum_required(VERSION 3.5)
project(yahboomcar_msgs)

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
find_package(rosidl_default_generators REQUIRED)  # gerador de código para msgs ROS2
find_package(geometry_msgs REQUIRED)              # geometry_msgs/Point usado em PointArray
find_package(std_msgs REQUIRED)                   # incluído mas não usado diretamente

# Declara as mensagens a gerar — adicionar aqui se criar novos .msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImageMsg.msg"
  "msg/PointArray.msg"
  "msg/Position.msg"
  "msg/Target.msg"
  "msg/TargetArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
