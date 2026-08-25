#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- 工具函数 ----------

// 指数映射：小旋转向量 δθ → 单位四元数
Eigen::Quaterniond exp_map(const Eigen::Vector3d& dtheta) {
    double angle = dtheta.norm();
    if (angle < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
}

// 反对称矩阵 [v]×（叉积的矩阵形式）
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0, -v.z(),  v.y(),
          v.z(),  0.0, -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

int main() {
    const double g = 9.81;
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // ---------- 读回仿真数据 ----------
    std::ifstream in(std::string(DATA_DIR) + "/imu_sim.bin", std::ios::binary);
    int N; double dt;
    in.read(reinterpret_cast<char*>(&N), sizeof(N));
    in.read(reinterpret_cast<char*>(&dt), sizeof(dt));
    std::vector<Eigen::Vector3d> gyro(N), acc(N);
    for (int i = 0; i < N; ++i) {
        in.read(reinterpret_cast<char*>(&gyro[i]), sizeof(gyro[i]));
        in.read(reinterpret_cast<char*>(&acc[i]),  sizeof(acc[i]));
    }
    in.close();

    // 真值（上帝视角，只用来评估，不喂给滤波器）
    auto truth_p = [&](int k) {
        double t = k * dt;
        return Eigen::Vector3d(5.0 * std::cos(0.5 * t),
                               5.0 * std::sin(0.5 * t), 0.0);
    };

    // ---------- ESKF 参数 ----------
    const double sigma_obs = 0.3;   // 位置观测噪声 std (m)
    const double sigma_g   = 0.01;  // 陀螺白噪声（和数据生成一致）
    const double sigma_a   = 0.1;   // 加速度计白噪声（和数据生成一致）
    const double sigma_bg  = 1e-4;  // 陀螺偏置随机游走（偏置是常数，取很小）
    const double sigma_ba  = 1e-3;  // 加速度计偏置随机游走
    const int    obs_every = 20;    // 每 20 步给一次位置观测（5Hz）

    std::mt19937 rng(7);
    std::normal_distribution<double> nd_obs(0.0, sigma_obs);

    // ---------- 大状态（Nominal State） ----------
    Eigen::Vector3d p(5.0, 0.0, 0.0);                      // 位置（圆周起点）
    Eigen::Vector3d v(0.0, 2.5, 0.0);                      // 速度（圆周切线方向）
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // 姿态（车头朝 x）
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();         // 陀螺偏置估计（从 0 学起）
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();         // 加速度计偏置估计（从 0 学起）

    // ---------- 纯积分对照（阶段1 的做法，不做任何校正） ----------
    Eigen::Vector3d p_p = p, v_p = v;
    Eigen::Quaterniond q_p = q;

    // ---------- 误差状态协方差 P（15×15，顺序：δp δv δθ δb_g δb_a） ----------
    Eigen::Matrix<double,15,15> P = Eigen::Matrix<double,15,15>::Zero();
    P.block<3,3>(0,0)   = 1e-4 * I3;          // 起始位置已知，很小
    P.block<3,3>(3,3)   = 1e-4 * I3;          // 起始速度已知
    P.block<3,3>(6,6)   = 1e-6 * I3;          // 起始姿态已知
    P.block<3,3>(9,9)   = 0.05*0.05 * I3;     // 陀螺偏置不知道，给大点让它学
    P.block<3,3>(12,12) = 0.20*0.20 * I3;     // 加速度计偏置不知道

    // 过程噪声 Q（每个误差量每步"被推着动"的方差）
    Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();
    Q.block<3,3>(3,3)   = (sigma_a*sigma_a*dt) * I3;   // δv 被加速度噪声驱动
    Q.block<3,3>(6,6)   = (sigma_g*sigma_g*dt) * I3;   // δθ 被陀螺噪声驱动
    Q.block<3,3>(9,9)   = (sigma_bg*sigma_bg*dt) * I3; // δb_g 随机游走
    Q.block<3,3>(12,12) = (sigma_ba*sigma_ba*dt) * I3; // δb_a 随机游走

    // 观测噪声 R（位置观测）
    Eigen::Matrix3d Rm = (sigma_obs*sigma_obs) * I3;

    // 观测矩阵 H（只观测位置 δp）
    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0,0) = I3;

    // ---------- 主循环 ----------
    double pure_sq = 0.0, eskf_sq = 0.0;
    std::cout << "  k | 纯积分误差 | ESKF误差 | 纯/ESKF\n";

    for (int i = 1; i < N; ++i) {
        // === ① 预测：用 IMU 积分大状态 ===
        // 用当前估计的偏置修正读数：真实角速度 = 测量 - 偏置
        Eigen::Vector3d w_B = gyro[i] - b_g;
        Eigen::Vector3d a_B = acc[i] - b_a;
        Eigen::Matrix3d R = q.toRotationMatrix();   // 当前姿态（body→world）

        // 机体系比力 → 世界系加速度（+重力）
        Eigen::Vector3d a_W = R * a_B + Eigen::Vector3d(0.0, 0.0, -g);

        p += v * dt + 0.5 * a_W * dt * dt;
        v += a_W * dt;
        q = q * exp_map(w_B * dt);     // 角速度积分（右乘指数映射）
        q.normalize();

        // === ② 误差状态预测：P ← F·P·Fᵀ + Q ===
        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();
        F.block<3,3>(0,3)  = I3 * dt;              // δp += δv·dt
        F.block<3,3>(3,6)  = -R * skew(a_B) * dt;  // δv ← -R[a]ₓ·δθ·dt
        F.block<3,3>(3,12) = -R * dt;              // δv ← -R·δb_a·dt
        F.block<3,3>(6,6)  = I3 - skew(w_B) * dt;  // δθ ← -[ω]ₓ·δθ·dt
        F.block<3,3>(6,9)  = -I3 * dt;             // δθ ← -δb_g·dt
        P = F * P * F.transpose() + Q;

        // === 纯积分对照（不校正，阶段1 的做法） ===
        Eigen::Vector3d a_pW = q_p.toRotationMatrix() * acc[i]
                               + Eigen::Vector3d(0,0,-g);
        p_p += v_p * dt + 0.5 * a_pW * dt * dt;
        v_p += a_pW * dt;
        q_p = q_p * exp_map(gyro[i] * dt);
        q_p.normalize();

        // === ③④⑤ 观测更新（每隔 obs_every 步来一次位置观测） ===
        if (i % obs_every == 0) {
            // 雷达给一个带噪声的位置观测
            Eigen::Vector3d z = truth_p(i) + Eigen::Vector3d(nd_obs(rng), nd_obs(rng), nd_obs(rng));

            // 新息：观测 - 预测的位置
            Eigen::Vector3d innov = z - p;

            // 卡尔曼增益（矩阵版）
            Eigen::Matrix<double,15,3> K =
                P * H.transpose() * (H * P * H.transpose() + Rm).inverse();

            // 误差状态修正量
            Eigen::Matrix<double,15,1> dx = K * innov;

            // ④ 把误差叠回大状态
            p   += dx.head<3>();                     // 位置误差
            v   += dx.segment<3>(3);                 // 速度误差
            q    = q * exp_map(dx.segment<3>(6));    // 姿态误差 δθ → 四元数叠回
            q.normalize();
            b_g += dx.segment<3>(9);                 // 陀螺偏置误差
            b_a += dx.segment<3>(12);                // 加速度计偏置误差

            // ⑤ 误差已用掉，协方差收缩
            P = (Eigen::Matrix<double,15,15>::Identity() - K * H) * P;
        }

        // ---- 记录误差 ----
        pure_sq += (p_p - truth_p(i)).squaredNorm();
        eskf_sq += (p - truth_p(i)).squaredNorm();

        // ---- 输出表格 ----
        if (i == 250 || i == 500 || i == 750 || i == 999) {
            double ep = (p_p - truth_p(i)).norm();
            double ee = (p - truth_p(i)).norm();
            std::cout << std::setw(4) << i << " | "
                      << std::fixed << std::setprecision(2)
                      << std::setw(8) << ep << " | "
                      << std::setw(8) << ee << " | "
                      << std::setw(6) << ep/ee << "\n";
        }
    }

    // ---------- 汇总 ----------
    double rmse_pure = std::sqrt(pure_sq / (N-1));
    double rmse_eskf = std::sqrt(eskf_sq / (N-1));
    std::cout << "\n全程位置 RMSE：纯积分 " << rmse_pure
              << " m，ESKF " << rmse_eskf << " m\n";
    std::cout << "ESKF 把位置误差平均压到了 " << rmse_pure/rmse_eskf << " 分之一\n";

    // 偏置估计（真值 b_g=[0.02,-0.03,0.05], b_a=[0.1,-0.2,0.3]）
    std::cout << "\n陀螺偏置估计     : " << b_g.transpose() << "   (真值 0.02 -0.03 0.05)\n";
    std::cout << "加速度计偏置估计 : " << b_a.transpose() << "   (真值 0.1 -0.2 0.3)\n";
    return 0;
}
