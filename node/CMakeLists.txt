cmake_minimum_required(VERSION 3.8)
project(omnihand_node)

# 设置标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 包含头文件目录
# include_directories(include)

add_subdirectory(node_msg)

# 创建可执行文件
add_executable(omnihand_node src/main.cpp src/hand_node.cpp)

# 设置包含目录
target_include_directories(
  omnihand_node
  PRIVATE ${CMAKE_SOURCE_DIR}/src
          ${USER_PATH})

# 指定依赖 - 必须在target_link_libraries之前
ament_target_dependencies(omnihand_node
  rclcpp
  std_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_cpp
)

# 获取消息类型支持目标并链接库
rosidl_get_typesupport_target(cpp_typesupport_target omnihand_node_msgs "rosidl_typesupport_cpp")

# 链接库 - 使用plain语法
target_link_libraries(omnihand_node 
  omnihand2025
  ${cpp_typesupport_target})

# 安装可执行文件和消息库
install(TARGETS omnihand_node DESTINATION bin)

# 安装Python脚本
install(PROGRAMS scripts/motor_pos_subscriber.py DESTINATION bin)

ament_package()