#pragma once
// ============================================================
//  共享 ESKF 核心（阶段6 起，stage6 和 ROS2 节点共用这一份）
//  数学和 stage4_eskf.cpp 完全一致，唯一区别：
//    predict() 接收"每步实际的 dt"（真实 IMU 时间戳不是均匀的）
// ============================================================
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eskf {

// ---------- 工具函数 ----------

// 指数映射：小旋转向量 δθ → 单位四元数
inline Eigen::Quaterniond exp_map(const Eigen::Vector3d& dtheta) {
    double angle = dtheta.norm();
    if (angle < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
}

// 反对称矩阵 [v]×（叉积的矩阵形式）
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0, -v.z(),  v.y(),
          v.z(),  0.0, -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

// ---------- 参数 ----------

// 过程噪声参数。离散 Q 块 = σ²·dt·I（和 stage4 同款形式）。
// 真实数据里 σ 直接填 sensor.yaml 的"噪声密度"：
//   连续白噪声 PSD = σ²，积分一步 dt 后等价离散方差 = σ²·dt，所以 σ := density。
struct NoiseParams {
    double sigma_g   = 1.7e-4;   // 陀螺白噪声密度  rad/s/√Hz
    double sigma_a   = 2e-4;     // 加速度计白噪声密度  m/s²/√Hz
    double sigma_bg  = 2e-4;     // 陀螺偏置随机游走  rad/√s
    double sigma_ba  = 2e-5;     // 加速度计偏置随机游走  m/s²/√s
    double sigma_obs = 0.1;      // 位置观测噪声 std (m)
};

// ---------- 大状态（Nominal State） ----------
struct State {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();            // 位置 (world)
    Eigen::Vector3d v = Eigen::Vector3d::Zero();            // 速度 (world)
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // 姿态 body→world
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();          // 陀螺偏置估计
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();          // 加速度计偏置估计
};

// ---------- 15 维误差状态 ESKF（顺序：δp δv δθ δb_g δb_a） ----------
class Eskf {
public:
    // 初始化：大状态从 init 起（真实数据用 GT 首样本；偏置从 0 学起）
    void reset(const State& init, const NoiseParams& np, double g = 9.81) {
        state_ = init;
        np_ = np;
        g_ = g;
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        P_ = Eigen::Matrix<double,15,15>::Zero();
        P_.block<3,3>(0,0)   = 1e-4 * I3;          // 起始位置已知，很小
        P_.block<3,3>(3,3)   = 1e-4 * I3;          // 起始速度已知
        P_.block<3,3>(6,6)   = 1e-6 * I3;          // 起始姿态已知
        P_.block<3,3>(9,9)   = 0.05*0.05 * I3;     // 陀螺偏置不知道，给大点让它学
        P_.block<3,3>(12,12) = 0.20*0.20 * I3;     // 加速度计偏置不知道
    }

    // 预测一步：w_raw=测量角速度(rad/s)，a_raw=测量比力(m/s²)，dt=本步实际时长(s)
    void predict(const Eigen::Vector3d& w_raw, const Eigen::Vector3d& a_raw, double dt) {
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

        // 用当前估计的偏置修正读数
        Eigen::Vector3d w_B = w_raw - state_.b_g;
        Eigen::Vector3d a_B = a_raw - state_.b_a;
        Eigen::Matrix3d R = state_.q.toRotationMatrix();   // body→world

        // 机体系比力 → 世界系加速度（+重力）
        Eigen::Vector3d a_W = R * a_B + Eigen::Vector3d(0.0, 0.0, -g_);

        // ① 名义状态积分（每步真实 dt）
        state_.p += state_.v * dt + 0.5 * a_W * dt * dt;
        state_.v += a_W * dt;
        state_.q = state_.q * exp_map(w_B * dt);
        state_.q.normalize();

        // ② 误差状态协方差预测 P ← F·P·Fᵀ + Q
        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();
        F.block<3,3>(0,3)  = I3 * dt;              // δp += δv·dt
        F.block<3,3>(3,6)  = -R * skew(a_B) * dt;  // δv ← -R[a]ₓ·δθ·dt
        F.block<3,3>(3,12) = -R * dt;              // δv ← -R·δb_a·dt
        F.block<3,3>(6,6)  = I3 - skew(w_B) * dt;  // δθ ← -[ω]ₓ·δθ·dt
        F.block<3,3>(6,9)  = -I3 * dt;             // δθ ← -δb_g·dt

        Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();
        Q.block<3,3>(3,3)   = (np_.sigma_a*np_.sigma_a*dt) * I3;
        Q.block<3,3>(6,6)   = (np_.sigma_g*np_.sigma_g*dt) * I3;
        Q.block<3,3>(9,9)   = (np_.sigma_bg*np_.sigma_bg*dt) * I3;
        Q.block<3,3>(12,12) = (np_.sigma_ba*np_.sigma_ba*dt) * I3;

        P_ = F * P_ * F.transpose() + Q;
    }

    // 位置观测更新：z_p = 世界系位置观测 (m)
    void update(const Eigen::Vector3d& z_p) {
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d innov = z_p - state_.p;   // 新息：观测 - 预测位置

        // 观测矩阵 H（只观测位置 δp）
        Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
        H.block<3,3>(0,0) = I3;

        // 卡尔曼增益
        Eigen::Matrix3d Rm = (np_.sigma_obs*np_.sigma_obs) * I3;
        Eigen::Matrix<double,15,3> K =
            P_ * H.transpose() * (H * P_ * H.transpose() + Rm).inverse();
        Eigen::Matrix<double,15,1> dx = K * innov;

        // ③ 把误差叠回大状态
        state_.p   += dx.head<3>();
        state_.v   += dx.segment<3>(3);
        state_.q    = state_.q * exp_map(dx.segment<3>(6));
        state_.q.normalize();
        state_.b_g += dx.segment<3>(9);
        state_.b_a += dx.segment<3>(12);

        // ④ 误差已用掉，协方差收缩
        P_ = (Eigen::Matrix<double,15,15>::Identity() - K * H) * P_;
    }

    const State& state() const { return state_; }
    const Eigen::Matrix<double,15,15>& P() const { return P_; }
    const NoiseParams& params() const { return np_; }

private:
    State state_;
    NoiseParams np_;
    double g_ = 9.81;
    Eigen::Matrix<double,15,15> P_;
};

} // namespace eskf
