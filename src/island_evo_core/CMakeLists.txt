cmake_minimum_required(VERSION 3.8)
project(island_evo_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# 1. 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Individual.msg"
  "msg/Packet.msg"
  DEPENDENCIES std_msgs
)

# 2. 编译 C++ 节点 (包含所有算法源文件)
add_executable(island_node 
  src/island_node.cpp 
  src/genetic_algorithm.cpp
  src/pso_algorithm.cpp
  src/de_algorithm.cpp
)

# 指定头文件路径
target_include_directories(island_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
ament_target_dependencies(island_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(island_node "${cpp_typesupport_target}")

# 3. 安装规则 (告诉 Docker 把编译好的东西放哪)
install(TARGETS island_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# 安装 Python 监控脚本
install(PROGRAMS
  src/monitor_node.py
  src/generation_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
