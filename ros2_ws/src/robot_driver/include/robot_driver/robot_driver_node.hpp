#ifndef ROBOT_DRIVER_NODE_HPP
#define ROBOT_DRIVER_NODE_HPP
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_driver/serial.hpp"
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
// ROS2底盘驱动节点
// 负责ROS2与STM32之间的数据转换
class RobotDriverNode : public rclcpp::Node
{
public:

    RobotDriverNode();
    ~RobotDriverNode();

private:

    // 串口对象，负责底层UART通信
    SerialPort serial_;
    float vx_;
    float vy_;
    float wz_;
    // 定时器，用于周期发送控制指令
    rclcpp::TimerBase::SharedPtr timer_;

    // 定时发送速度命令
    void sendCommand();
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ==========================================================
    // 底盘状态上传相关的成员函数 (STM32 -> ROS2)
    // ==========================================================

    // 后台独立线程函数：专门负责死循环读取 STM32 通过串口发来的数据。
    // 读到以 "ODOM" 开头的数据后，会进行字符串解析并提取出6个数值。
    void receiveDataLoop();

    // 核心发布函数：将解析出的数据打包成 ROS2 标准格式并发布。
    // 参数 vx, vy, wz: 机器人的线速度和角速度（用于发布 Odometry 里的 twist 速度信息）
    // 参数 x, y, theta: 机器人在世界坐标系下的累计位置和偏航角（用于发布 TF 和 Odometry 里的 pose 位置信息）
    void publishOdomAndTF(float vx, float vy, float wz, float x, float y, float theta);

    // 发布 IMU 数据：解析 "IMU,ax,ay,az,gx,gy,gz,qw,qx,qy,qz" 行并打包成 sensor_msgs::msg::Imu。
    // 单位约定: 加速度 m/s^2, 角速度 rad/s, 四元数无量纲。
    // STM32 端 gx/gy/ax/ay 是芯片原始坐标系, Madgwick 解算四元数时已做重映射
    // (gy,-gx,gz) 和 (ay,-ax,az) 得到 FLU 坐标, 因此这里把角速度和加速度同样映射到 FLU,
    // 保证与四元数、与 odom 的 base_link 坐标系一致。
    void publishImu(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float qw, float qx, float qy, float qz);


    // ==========================================================
    // 底盘状态上传相关的成员变量
    // ==========================================================

    // 里程计话题发布器：负责向 "/odom" 话题发送 nav_msgs::msg::Odometry 消息，
    // 供 Nav2 和 SLAM 算法订阅，让系统知道机器人的实时速度和累计里程。
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // IMU 话题发布器：向 "/imu/data" 发送 sensor_msgs::msg::Imu，
    // 供 robot_localization 的 ekf_node 订阅做多传感器融合。
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // TF 坐标变换广播器：负责在 ROS2 系统中发布动态坐标树。
    // 具体作用是建立 "odom" (世界基准) 到 "base_link" (机器人车体) 的空间位置关系，
    // 只有发布了这个，RViz 里才能看到小车移动。
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 是否广播 odom->base_link TF。默认 true 保持原有行为；
    // 使用 EKF 时设为 false，由 ekf_node 统一广播融合后的 TF，避免双源冲突。
    bool publish_tf_;

    // IMU 消息协方差对角值（sensor_msgs/Imu 要求非零，ekf_node 才会融合对应测量）
    double imu_orient_cov_;
    double imu_angular_vel_cov_;
    double imu_linear_acc_cov_;

    // 独立读取线程对象：因为 ROS2 的 spin 机制在处理下发命令，
    // 为了防止读取串口阻塞 ROS2 的正常通讯，必须开辟一个 C++ 独立线程来运行 receiveDataLoop()。
    std::thread receive_thread_;

};

#endif
