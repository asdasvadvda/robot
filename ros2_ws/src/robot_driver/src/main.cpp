#include "rclcpp/rclcpp.hpp"
#include "robot_driver/robot_driver_node.hpp"


int main(int argc,char **argv)
{

    // 初始化ROS2
    rclcpp::init(argc,argv);


    // 创建节点
    auto node = std::make_shared<RobotDriverNode>();


    // 循环运行节点
    rclcpp::spin(node);


    // 关闭ROS2
    rclcpp::shutdown();


    return 0;
}
