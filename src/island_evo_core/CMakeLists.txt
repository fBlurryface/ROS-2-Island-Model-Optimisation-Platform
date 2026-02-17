cmake_minimum_required(VERSION 3.8)
project(island_evo_core)

# 开启 C++17 以支持 std::filesystem
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Individual.msg"
  "msg/Packet.msg"
  DEPENDENCIES std_msgs
)

# 3. 编译 C++ 节点
add_executable(island_node
  src/island_node.cpp
  src/genetic_algorithm.cpp
  src/pso_algorithm.cpp
  src/de_algorithm.cpp
)

target_include_directories(island_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Jazzy 专用：正确链接消息
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(island_node "${cpp_typesupport_target}")

ament_target_dependencies(island_node rclcpp std_msgs)

# 4. 安装规则
install(TARGETS island_node
  DESTINATION lib/${PROJECT_NAME})

# 确保指向 scripts/ 目录
install(PROGRAMS
  scripts/generation_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()