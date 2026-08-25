#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Core>

int main() {
    // ---- 读回数据（和之前一样） ----
    std::ifstream in(std::string(DATA_DIR) + "/imu_sim.bin", std::ios::binary);
    int    N;
    double dt;
    in.read(reinterpret_cast<char*>(&N),  sizeof(N));
    in.read(reinterpret_cast<char*>(&dt), sizeof(dt));
    std::vector<Eigen::Vector3d> gyro(N), acc(N);
    for (int i = 0; i < N; ++i) {
        in.read(reinterpret_cast<char*>(&gyro[i]), sizeof(gyro[i]));
        in.read(reinterpret_cast<char*>(&acc[i]),  sizeof(acc[i]));
    }
    in.close();

    const double g = 9.81;

    // ---- 状态：从零开始 ----
    std::vector<double>         psi(N, 0.0);      // 偏航角
    std::vector<Eigen::Matrix3d> R_hat(N);        // 歪的姿态（从 psi 重建）
    std::vector<Eigen::Vector3d> v(N, Eigen::Vector3d::Zero());   // 速度
    std::vector<Eigen::Vector3d> p(N, Eigen::Vector3d::Zero());   // 位置

    // ---- 初始姿态：世界系，不歪 ----
    R_hat[0] = Eigen::Matrix3d::Identity();

    for (int i = 1; i < N; ++i) {
        // ① 偏航角积分
        psi[i] = psi[i-1] + gyro[i].z() * dt;

        // ② 从 psi 重建歪的姿态（2D 旋转矩阵，和阶段0真值 R_WB 一样的结构）
        double c = std::cos(psi[i]), s = std::sin(psi[i]);
        R_hat[i] << c,  s, 0.0,
                   -s,  c, 0.0,
                    0.0, 0.0, 1.0;

        // ③④ 机体系加速度 = 比力(acc) + 机体系重力
        Eigen::Vector3d g_B = R_hat[i] * Eigen::Vector3d(0.0, 0.0, -g);
        Eigen::Vector3d a_B = acc[i] + g_B;

        // ⑤ 转回世界系：R_hatᵀ 是 R_hat 的逆（旋转矩阵的逆=转置）
        Eigen::Vector3d a_W = R_hat[i].transpose() * a_B;

        // ⑥⑦ 积分速度、位置
        v[i] = v[i-1] + a_W * dt;
        p[i] = p[i-1] + v[i] * dt;
    }

    // ---- 输出：对比真值和积分位置 ----
    std::cout << " 时刻  真值位置(x,y)          积分位置(x,y)           误差\n";
    for (int k : {0, 250, 500, 999}) {
        double t = k * dt;
        double p_true_x = 5.0 * std::cos(0.5 * t);
        double p_true_y = 5.0 * std::sin(0.5 * t);
        double err = (p[k].head<2>() - Eigen::Vector2d(p_true_x, p_true_y)).norm();
        std::cout << "k=" << k << "   (" << p_true_x << "," << p_true_y
                  << ")     (" << p[k].x() << "," << p[k].y()
                  << ")     误差=" << err << "\n";
    }
    return 0;
}
