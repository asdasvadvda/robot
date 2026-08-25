#include "robot_driver/robot_driver_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

RobotDriverNode::RobotDriverNode()
: Node("robot_driver_node")
{

    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(
            &RobotDriverNode::cmdVelCallback,
            this,
            std::placeholders::_1
        )
    );
    // 打开STM32串口
    // 注意这里的串口名后面根据实际情况修改
    if(!serial_.openPort("/dev/ttyACM0",115200))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Serial open failed!"
        );
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Serial opened successfully"
        );
    }


    // 创建100Hz定时器
    // 与STM32当前100Hz控制循环保持一致
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(
            &RobotDriverNode::sendCommand,
            this
        )
    );
    //Odom 和 TF 初始化 (处理上传)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // IMU 发布器, 供 robot_localization 的 ekf_node 订阅
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 参数: 是否广播 odom->base_link TF (EKF 接管时应设为 false)
    publish_tf_ = this->declare_parameter("publish_tf", true);
    // IMU 协方差对角值, 非零 ekf_node 才会融合对应测量 (对角占优即可)
    imu_orient_cov_ = this->declare_parameter("imu_orientation_covariance", 0.001);
    imu_angular_vel_cov_ = this->declare_parameter("imu_angular_velocity_covariance", 0.001);
    imu_linear_acc_cov_ = this->declare_parameter("imu_linear_acceleration_covariance", 0.01);

    // 启动独立线程读取 STM32 数据
    receive_thread_ = std::thread(&RobotDriverNode::receiveDataLoop, this);

}

// === 注意：在析构函数中要等待线程结束，防止节点退出时崩溃 ===
RobotDriverNode::~RobotDriverNode()
{
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

void RobotDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
}

void RobotDriverNode::sendCommand()
{

    // 测试发送固定速度
    // 格式必须匹配STM32:
    // CMD,vx,vy,wz

    char buffer[100];

    sprintf(buffer,
            "CMD,%.3f,%.3f,%.3f\n",
            vx_,
            vy_,
            wz_);

    serial_.writeData(buffer);


    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Sending command: %s",
        buffer
    );

}

// 串口读取线程 (带分包拼接逻辑)
void RobotDriverNode::receiveDataLoop()
{
    std::string rx_buffer = ""; // 数据拼接池

    while (rclcpp::ok())
    {
        // 1. 读取当前串口里的任意长度碎片
        std::string chunk = serial_.readData(); 

        if (!chunk.empty())
        {
            // 2. 把新读到的碎片拼接到池子里
            rx_buffer += chunk; 

            // 3. 检查池子里有没有完整的行（寻找换行符 '\n'）
            size_t pos;
            while ((pos = rx_buffer.find('\n')) != std::string::npos)
            {
                // 截取出一整行完整数据 (不包含 '\n')
                std::string line = rx_buffer.substr(0, pos);
                
                // 将处理完的这行数据（连同 '\n'）从池子里删掉
                rx_buffer.erase(0, pos + 1);
                
                // --- 下面就是你熟悉的解析逻辑 ---
                if (!line.empty() && line.rfind("ODOM", 0) == 0)
                {
                    float vx, vy, wz, x, y, theta;
                    int parsed = sscanf(line.c_str(), "ODOM,%f,%f,%f,%f,%f,%f",
                                        &vx, &vy, &wz, &x, &y, &theta);

                    if (parsed == 6) {
                        publishOdomAndTF(vx, vy, wz, x, y, theta);
                    }
                }
                else if (!line.empty() && line.rfind("IMU", 0) == 0)
                {
                    // IMU,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
                    float ax, ay, az, gx, gy, gz, qw, qx, qy, qz;
                    int parsed = sscanf(line.c_str(), "IMU,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                        &ax, &ay, &az, &gx, &gy, &gz, &qw, &qx, &qy, &qz);

                    if (parsed == 10) {
                        publishImu(ax, ay, az, gx, gy, gz, qw, qx, qy, qz);
                    }
                }
            }
        }
        
        // 稍微休眠，防止死循环占满单核 CPU 
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 发布 TF 和 Odom 话题
void RobotDriverNode::publishOdomAndTF(float vx, float vy, float wz, float x, float y, float theta)
{
    auto current_time = this->get_clock()->now();

    // 1. 偏航角转四元数
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);

    // 2. 发布 TF: odom -> base_link
    // 仅当 publish_tf_ 为 true 时广播; 使用 EKF 时由 ekf_node 广播融合后的 TF,
    // 避免同一变换有两个发布源造成 TF 抖动。
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }

    // 3. 发布 /odom
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = wz;

    odom_pub_->publish(odom_msg);
}

// 发布 IMU 消息 (imu/data)
// 输入行格式: IMU,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
// 单位约定 (STM32 固件侧): 加速度 m/s^2, 角速度 rad/s, 四元数无量纲。
//
// 坐标轴说明:
//   四元数 qw/qx/qy/qz 已是 FLU (base_link 同系, 与 odom 一致);
//   但 gx/gy/ax/ay 是芯片原始坐标系。STM32 端 Madgwick 解算时用
//   (gy,-gx,gz) 和 (ay,-ax,az) 作为 FLU 输入, 因此这里把角速度和加速度
//   做同样的重映射, 保证整条 Imu 消息都在 FLU 坐标系下, 与四元数自洽。
void RobotDriverNode::publishImu(float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float qw, float qx, float qy, float qz)
{
    auto current_time = this->get_clock()->now();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "base_link";

    // 姿态四元数 (STM32 Madgwick 解算, 已映射到 FLU)
    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;

    // 角速度: 芯片系 -> FLU: (gx,gy,gz) -> (gy, -gx, gz), 单位 rad/s
    imu_msg.angular_velocity.x = gy;
    imu_msg.angular_velocity.y = -gx;
    imu_msg.angular_velocity.z = gz;

    // 线加速度: 芯片系 -> FLU: (ax,ay,az) -> (ay, -ax, az), 单位 m/s^2
    imu_msg.linear_acceleration.x = ay;
    imu_msg.linear_acceleration.y = -ax;
    imu_msg.linear_acceleration.z = az;

    // 协方差必须非零, robot_localization 才会融合对应测量
    // 对角占优即可, 数值可在 launch 里通过参数调整
    imu_msg.orientation_covariance[0] = imu_orient_cov_;
    imu_msg.orientation_covariance[4] = imu_orient_cov_;
    imu_msg.orientation_covariance[8] = imu_orient_cov_;

    imu_msg.angular_velocity_covariance[0] = imu_angular_vel_cov_;
    imu_msg.angular_velocity_covariance[4] = imu_angular_vel_cov_;
    imu_msg.angular_velocity_covariance[8] = imu_angular_vel_cov_;

    imu_msg.linear_acceleration_covariance[0] = imu_linear_acc_cov_;
    imu_msg.linear_acceleration_covariance[4] = imu_linear_acc_cov_;
    imu_msg.linear_acceleration_covariance[8] = imu_linear_acc_cov_;

    imu_pub_->publish(imu_msg);
}
