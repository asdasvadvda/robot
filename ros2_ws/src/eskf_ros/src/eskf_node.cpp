// ============================================================
//  eskf_node：把 ESKF 学习项目包成 ROS2 节点
//
//  订阅：/imu  (sensor_msgs/Imu)     IMU 角速度 + 比力（~200Hz）
//        /odom (nav_msgs/Odometry)   外部位置观测（低频，如轮速/雷达/VO）
//  发布：/odom_filtered (nav_msgs/Odometry)  滤波后的高频位姿（~IMU 频率）
//
//  教学要点：
//   ① Imu.linear_acceleration 是"比力"（specific force），不是运动加速度。
//      静止时它读 +g。ESKF 内部会做 R·a_B + (0,0,-g)，节点里**不要**再减重力。
//   ② 两个传感器各有自己的时钟，必须按 header.stamp 对齐：
//      先把观测塞进队列，等 IMU 时间赶上观测时间才消费（时间序）。
//   ③ dt 用相邻 IMU 的 header.stamp 差值算（真实数据不是均匀采样）。
//
//  运行：source install/setup.bash && ros2 run eskf_ros eskf_node
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <deque>
#include <memory>

#include "eskf.hpp"

class EskfNode : public rclcpp::Node {
public:
    EskfNode() : Node("eskf_node") {
        // ---- 参数（可 tune，教学用） ----
        this->declare_parameter<double>("sigma_obs", 0.1);   // 位置观测噪声 std (m)
        this->declare_parameter<double>("sigma_g", 1.7e-4);  // 陀螺噪声密度
        this->declare_parameter<double>("sigma_a", 2e-4);    // 加速度计噪声密度
        this->declare_parameter<double>("sigma_bg", 2e-4);   // 陀螺偏置随机游走
        this->declare_parameter<double>("sigma_ba", 2e-5);   // 加速度计偏置随机游走
        this->declare_parameter<double>("gravity", 9.81);    // 重力大小
        this->declare_parameter<std::string>("frame_world", "world");
        this->declare_parameter<std::string>("frame_body", "base_link");

        np_.sigma_obs = this->get_parameter("sigma_obs").as_double();
        np_.sigma_g   = this->get_parameter("sigma_g").as_double();
        np_.sigma_a   = this->get_parameter("sigma_a").as_double();
        np_.sigma_bg  = this->get_parameter("sigma_bg").as_double();
        np_.sigma_ba  = this->get_parameter("sigma_ba").as_double();
        g_ = this->get_parameter("gravity").as_double();
        world_frame_ = this->get_parameter("frame_world").as_string();
        body_frame_  = this->get_parameter("frame_body").as_string();

        // ---- 话题 ----
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::QoS(50),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(50),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_filtered", rclcpp::QoS(50));

        RCLCPP_INFO(this->get_logger(),
                    "eskf_node 就绪：订阅 /imu + /odom，发布 /odom_filtered");
    }

private:
    // ---------- /odom 回调：进队列；第一个 odom 触发初始化 ----------
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pending_odom_.push_back(msg);
        if (!init_) {
            // 用第一个观测当初始位姿（世界系）
            eskf::State s0;
            s0.p << msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z;
            s0.q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
            // 初始速度：优先用 odom 给的线速度（世界系）
            s0.v << msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z;
            // 偏置从 0 学起
            eskf_.reset(s0, np_, g_);
            last_obs_stamp_ = rclcpp::Time(msg->header.stamp);
            init_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "已用第一个 /odom 初始化：p=[%.2f %.2f %.2f]",
                        s0.p.x(), s0.p.y(), s0.p.z());
        }
    }

    // ---------- /imu 回调：预测 → 按时间消费观测 → 发布 ----------
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!init_) return;   // 还没有位置，先等着

        // 第一步没有"前一个 IMU"算 dt，只记录时间戳
        if (!have_imu_) {
            last_imu_stamp_ = rclcpp::Time(msg->header.stamp);
            have_imu_ = true;
            return;
        }

        // dt = 相邻 IMU 时间戳差（真实数据非均匀）
        // 注意：两者都得是 ROS 时间源（消息时间戳），不能和默认构造混
        double dt = (rclcpp::Time(msg->header.stamp) - last_imu_stamp_).seconds();
        if (dt <= 0.0) return;          // 乱序/重复，跳过
        if (dt > 0.1) dt = 0.1;         // 大缺口封顶，防协方差爆掉

        // 预测：角速度、比力（注意：不再减重力！）
        eskf_.predict(
            Eigen::Vector3d(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z),
            Eigen::Vector3d(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z),
            dt);
        last_imu_stamp_ = rclcpp::Time(msg->header.stamp);

        // 消费"时间已到"的位置观测：stamp 不晚于当前 IMU 时刻
        while (!pending_odom_.empty()) {
            auto& m = pending_odom_.front();
            if (rclcpp::Time(m->header.stamp) > rclcpp::Time(msg->header.stamp)) break;  // 还没到，留着
            if (rclcpp::Time(m->header.stamp) > last_obs_stamp_) {   // 时间序消费，丢乱序旧的
                Eigen::Vector3d z(m->pose.pose.position.x,
                                  m->pose.pose.position.y,
                                  m->pose.pose.position.z);
                eskf_.update(z);
                last_obs_stamp_ = rclcpp::Time(m->header.stamp);
            }
            pending_odom_.pop_front();
        }

        // 发布滤波结果（IMU 频率，全速率轨迹）
        publish_odom(msg->header.stamp);
    }

    void publish_odom(const builtin_interfaces::msg::Time& stamp) {
        auto out = nav_msgs::msg::Odometry();
        out.header.stamp = stamp;
        out.header.frame_id = world_frame_;
        out.child_frame_id = body_frame_;

        const auto& s = eskf_.state();
        out.pose.pose.position.x = s.p.x();
        out.pose.pose.position.y = s.p.y();
        out.pose.pose.position.z = s.p.z();
        out.pose.pose.orientation.x = s.q.x();
        out.pose.pose.orientation.y = s.q.y();
        out.pose.pose.orientation.z = s.q.z();
        out.pose.pose.orientation.w = s.q.w();

        out.twist.twist.linear.x = s.v.x();
        out.twist.twist.linear.y = s.v.y();
        out.twist.twist.linear.z = s.v.z();
        // 角速度可补：R·(gyro - b_g)，这里先留 0（教学简化）

        // 协方差对角（位置）：来自 P 的位置块
        const auto& P = eskf_.P();
        out.pose.covariance[0]  = P(0,0);
        out.pose.covariance[7]  = P(1,1);
        out.pose.covariance[14] = P(2,2);

        pub_odom_->publish(out);
    }

    // ---------- 成员 ----------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    eskf::Eskf eskf_;
    eskf::NoiseParams np_;
    double g_ = 9.81;
    std::string world_frame_, body_frame_;

    std::deque<nav_msgs::msg::Odometry::SharedPtr> pending_odom_;
    bool init_ = false;      // 是否已用第一个 /odom 初始化
    bool have_imu_ = false;  // 是否收到过第一个 /imu
    rclcpp::Time last_imu_stamp_, last_obs_stamp_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EskfNode>());
    rclcpp::shutdown();
    return 0;
}
