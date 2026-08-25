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

Eigen::Quaterniond exp_map(const Eigen::Vector3d& dtheta) {
    double angle = dtheta.norm();
    if (angle < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0, -v.z(),  v.y(),
          v.z(),  0.0, -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

// 真值位置（上帝视角，评估用）
Eigen::Vector3d truth_p(int i, double dt) {
    double t = i * dt;
    return Eigen::Vector3d(5.0 * std::cos(0.5 * t),
                           5.0 * std::sin(0.5 * t), 0.0);
}

// ---------- 纯积分（阶段1 做法，不校正） ----------
std::vector<Eigen::Vector3d> run_pure(const std::vector<Eigen::Vector3d>& gyro,
                                      const std::vector<Eigen::Vector3d>& acc,
                                      double dt) {
    const double g = 9.81;
    const int N = static_cast<int>(gyro.size());
    std::vector<Eigen::Vector3d> traj(N);
    Eigen::Vector3d p(5.0, 0.0, 0.0), v(0.0, 2.5, 0.0);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    traj[0] = p;
    for (int i = 1; i < N; ++i) {
        Eigen::Vector3d a_W = q.toRotationMatrix() * acc[i]
                              + Eigen::Vector3d(0, 0, -g);
        p += v * dt + 0.5 * a_W * dt * dt;
        v += a_W * dt;
        q = q * exp_map(gyro[i] * dt);
        q.normalize();
        traj[i] = p;
    }
    return traj;
}

// ---------- ESKF（阶段4 做法） ----------
std::vector<Eigen::Vector3d> run_eskf(const std::vector<Eigen::Vector3d>& gyro,
                                      const std::vector<Eigen::Vector3d>& acc,
                                      double dt, int obs_every,
                                      Eigen::Vector3d& b_g_out,
                                      Eigen::Vector3d& b_a_out) {
    const double g = 9.81;
    const int N = static_cast<int>(gyro.size());
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    const double sigma_obs = 0.3;
    const double sigma_g = 0.01;
    const double sigma_a = 0.1;
    const double sigma_bg = 1e-4;
    const double sigma_ba = 1e-3;
    std::mt19937 rng(7);
    std::normal_distribution<double> nd_obs(0.0, sigma_obs);

    Eigen::Vector3d p(5.0, 0.0, 0.0), v(0.0, 2.5, 0.0);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();

    Eigen::Matrix<double,15,15> P = Eigen::Matrix<double,15,15>::Zero();
    P.block<3,3>(0,0)   = 1e-4 * I3;
    P.block<3,3>(3,3)   = 1e-4 * I3;
    P.block<3,3>(6,6)   = 1e-6 * I3;
    P.block<3,3>(9,9)   = 0.05*0.05 * I3;
    P.block<3,3>(12,12) = 0.20*0.20 * I3;

    Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();
    Q.block<3,3>(3,3)   = (sigma_a*sigma_a*dt) * I3;
    Q.block<3,3>(6,6)   = (sigma_g*sigma_g*dt) * I3;
    Q.block<3,3>(9,9)   = (sigma_bg*sigma_bg*dt) * I3;
    Q.block<3,3>(12,12) = (sigma_ba*sigma_ba*dt) * I3;

    Eigen::Matrix3d Rm = (sigma_obs*sigma_obs) * I3;
    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0,0) = I3;

    std::vector<Eigen::Vector3d> traj(N);
    traj[0] = p;
    for (int i = 1; i < N; ++i) {
        Eigen::Vector3d w_B = gyro[i] - b_g;
        Eigen::Vector3d a_B = acc[i] - b_a;
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d a_W = R * a_B + Eigen::Vector3d(0.0, 0.0, -g);
        p += v * dt + 0.5 * a_W * dt * dt;
        v += a_W * dt;
        q = q * exp_map(w_B * dt);
        q.normalize();

        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();
        F.block<3,3>(0,3)  = I3 * dt;
        F.block<3,3>(3,6)  = -R * skew(a_B) * dt;
        F.block<3,3>(3,12) = -R * dt;
        F.block<3,3>(6,6)  = I3 - skew(w_B) * dt;
        F.block<3,3>(6,9)  = -I3 * dt;
        P = F * P * F.transpose() + Q;

        if (i % obs_every == 0) {
            Eigen::Vector3d z = truth_p(i, dt) + Eigen::Vector3d(nd_obs(rng), nd_obs(rng), nd_obs(rng));
            Eigen::Vector3d innov = z - p;
            Eigen::Matrix<double,15,3> K =
                P * H.transpose() * (H * P * H.transpose() + Rm).inverse();
            Eigen::Matrix<double,15,1> dx = K * innov;
            p   += dx.head<3>();
            v   += dx.segment<3>(3);
            q    = q * exp_map(dx.segment<3>(6));
            q.normalize();
            b_g += dx.segment<3>(9);
            b_a += dx.segment<3>(12);
            P = (Eigen::Matrix<double,15,15>::Identity() - K * H) * P;
        }
        traj[i] = p;
    }
    b_g_out = b_g;
    b_a_out = b_a;
    return traj;
}

// ---------- 指标 ----------
// ATE：全程绝对轨迹误差
double ate(const std::vector<Eigen::Vector3d>& est,
           const std::vector<Eigen::Vector3d>& tru) {
    double s = 0.0;
    for (size_t i = 0; i < est.size(); ++i)
        s += (est[i] - tru[i]).squaredNorm();
    return std::sqrt(s / est.size());
}

// RPE：固定间隔 Δ 的相对位姿误差
double rpe(const std::vector<Eigen::Vector3d>& est,
           const std::vector<Eigen::Vector3d>& tru, int delta) {
    double s = 0.0; int m = 0;
    for (size_t i = 0; i + delta < est.size(); ++i) {
        Eigen::Vector3d T  = tru[i + delta] - tru[i];
        Eigen::Vector3d T_ = est[i + delta] - est[i];
        s += (T - T_).squaredNorm();
        ++m;
    }
    return std::sqrt(s / m);
}

int main() {
    // 读回仿真数据
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

    // 跑纯积分 + ESKF
    auto pure = run_pure(gyro, acc, dt);
    Eigen::Vector3d b_g, b_a;
    auto eskf = run_eskf(gyro, acc, dt, 20, b_g, b_a);

    // 真值轨迹
    std::vector<Eigen::Vector3d> tru(N);
    for (int i = 0; i < N; ++i) tru[i] = truth_p(i, dt);

    // 算指标
    int delta = static_cast<int>(1.0 / dt);   // RPE 间隔 = 1 秒
    double ate_p = ate(pure, tru), ate_e = ate(eskf, tru);
    double rpe_p = rpe(pure, tru, delta), rpe_e = rpe(eskf, tru, delta);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "========== ATE / RPE 评估 ==========\n";
    std::cout << "ATE (绝对轨迹误差) : 纯积分 " << std::setw(8) << ate_p
              << " m | ESKF " << std::setw(8) << ate_e
              << " m | 改善 " << ate_p / ate_e << " 倍\n";
    std::cout << "RPE (相对位姿误差) : 纯积分 " << std::setw(8) << rpe_p
              << " m | ESKF " << std::setw(8) << rpe_e
              << " m | 改善 " << rpe_p / rpe_e << " 倍\n";
    std::cout << "陀螺偏置估计 : " << b_g.transpose()
              << " (真值 0.02 -0.03 0.05)\n";
    std::cout << "加速偏置估计 : " << b_a.transpose()
              << " (真值 0.1 -0.2 0.3)\n";

    // 导出三条轨迹到 CSV（供 Python 画图）
    std::ofstream out(std::string(DATA_DIR) + "/traj.csv");
    out << "k,t,true_x,true_y,true_z,pure_x,pure_y,pure_z,eskf_x,eskf_y,eskf_z\n";
    for (int i = 0; i < N; ++i) {
        out << i << "," << i * dt << ","
            << tru[i].x() << "," << tru[i].y() << "," << tru[i].z() << ","
            << pure[i].x() << "," << pure[i].y() << "," << pure[i].z() << ","
            << eskf[i].x() << "," << eskf[i].y() << "," << eskf[i].z() << "\n";
    }
    out.close();
    std::cout << "\n已导出 data/traj.csv（三条轨迹，供画图）\n";
    return 0;
}
