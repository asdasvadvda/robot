cmake_minimum_required(VERSION 3.8)
project(eskf_ros)

# Humble 的 rclcpp ABI 按 C++17 编译，保持一致
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 复用 ESKF 仓库的共享核心 include/eskf.hpp + third_party/eigen。
# 用相对路径（ros2_ws/src/eskf_ros → 上两级再进 ESKF 即仓库根），仓库挪位置也不用改。
set(ESKF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../ESKF")
include_directories(${ESKF_ROOT}/include)
include_directories(${ESKF_ROOT}/third_party/eigen)

add_executable(eskf_node src/eskf_node.cpp)
ament_target_dependencies(eskf_node rclcpp sensor_msgs nav_msgs geometry_msgs)

install(TARGETS
  eskf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch.py")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
