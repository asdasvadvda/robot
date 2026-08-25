cmake_minimum_required(VERSION 3.8)

# ROS2包名称
project(robot_driver)


# 使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 添加头文件路径
include_directories(
  include
)


# 创建ROS2节点可执行文件
add_executable(robot_driver_node

  # 串口底层代码
  src/serial.cpp

  # ROS2节点代码
  src/robot_driver_node.cpp

  # main函数
  src/main.cpp

)


# 链接ROS2库
ament_target_dependencies(
  robot_driver_node
  rclcpp
)


#安装Twist消息数据流
ament_target_dependencies(
 robot_driver_node
 rclcpp
 geometry_msgs
)

ament_target_dependencies(
  robot_driver_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装配置和 launch 文件
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


# 安装可执行文件
install(
  TARGETS
    robot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
