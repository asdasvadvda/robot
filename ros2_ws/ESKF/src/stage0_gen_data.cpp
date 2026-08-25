#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <fstream>
#include <random> 
int main() {
    // ---- 仿真参数 ----
    const double dt      = 0.01;    // 采样周期 10ms
    const double T       = 10.0;    // 仿真时长 10s
    //static_cast<int> —— 显式类型转换（C 风格是 (int)(T/dt))
    //现代 C++ 用它，意图更明确
    const int    N       = static_cast<int>(T / dt);   // 1000 个采样点
    const double R_c     = 5.0;     // 半径 5m
    const double omega_c = 0.5;     // 角速度 0.5 rad/s
    const double sigma_g = 0.01;    // 陀螺白噪声 std：0.01 rad/s
    const double sigma_a = 0.1;     // 加速度计白噪声 std：0.1 m/s²
    std::mt19937 rng(42);                              // ① 随机数"机器"
    std::normal_distribution<double> nd_g(0.0, sigma_g);  // ② 高斯"骰子"
    std::normal_distribution<double> nd_a(0.0, sigma_a);  // ③ 高斯"骰子"
    Eigen::Vector3d b_g(0.02, -0.03, 0.05);        // 陀螺偏置 rad/s
    Eigen::Vector3d b_a(0.1, -0.2, 0.3);           // 加速度计偏置 m/s²
    // ---- 数据容器：装 N 个三维向量 ----
    std::vector<Eigen::Vector3d> p_W(N);   // 位置

    // ---- 生成圆周轨迹 ----
    for (int i = 0; i < N; ++i) {
        // 计算第 i 个采样点的相位角 theta
        double theta = omega_c * i * dt;   // 相位角，等价 Python 的 theta[i]
        p_W[i] = Eigen::Vector3d(R_c * std::cos(theta),
                                R_c * std::sin(theta),
                                0.0);
    }

    // ---- 验证：打印第 0、250、500 个点 ----
    for (int i : {0, 250, 500}) {
        std::cout << "p_W[" << i << "] = " << p_W[i].transpose() << "\n";
    }
        // ---- 速度：位置对时间求导（解析，无误差） ----
    std::vector<Eigen::Vector3d> v_W(N);
    for (int i = 0; i < N; ++i) {
        double theta = omega_c * i * dt;
        v_W[i] = Eigen::Vector3d(-R_c * omega_c * std::sin(theta),
                                R_c * omega_c * std::cos(theta),
                                0.0);
    }

    // ---- 加速度：速度对时间求导（解析） ----
    std::vector<Eigen::Vector3d> a_W(N);
    for (int i = 0; i < N; ++i) {
        double theta = omega_c * i * dt;
        a_W[i] = Eigen::Vector3d(-R_c * omega_c * omega_c * std::cos(theta),
                                -R_c * omega_c * omega_c * std::sin(theta),
                                0.0);
    }

    // ---- 姿态：机头沿切线方向（偏航角 = 相位角），R_WB 把世界系搬到机体系 ----
    std::vector<Eigen::Matrix3d> R_WB(N);
    for (int i = 0; i < N; ++i) {
        double c = std::cos(omega_c * i * dt);
        double s = std::sin(omega_c * i * dt);
        R_WB[i] << c,  s, 0.0,
                  -s,  c, 0.0,
                    0.0, 0.0, 1.0;
    }
    // ---- 角速度 omega_B：从姿态导数求（机体系） ----
    // 数学依据：omega_B = R_WB · omega_W，绕世界 z 轴转时 omega_W = [0,0,0.5]
    // 所以 omega_B = R_WB · [0,0,0.5]^T = [0, 0, 0.5]（偏航旋转不改变 z 分量）
    Eigen::Vector3d omega_W(0.0, 0.0, omega_c);
    std::vector<Eigen::Vector3d> omega_B(N);
    for (int i = 0; i < N; ++i) {
        omega_B[i] = R_WB[i] * omega_W;   // 把世界系角速度转到机体系
    }
    // ---- 世界系比力：f_W = a_W - g_W（加速度计"感知"的东西） ----
    const Eigen::Vector3d g_W(0.0, 0.0, -9.81);// 世界系重力
    std::vector<Eigen::Vector3d> f_W(N);
    for (int i = 0; i < N; ++i) {
        f_W[i] = a_W[i] - g_W;
    }
    // ---- 验证输出：看几个关键值是否和手算一致 ----
    int k = 250;// 挑第 250 个时刻
    std::cout << "p_W[k]    = " << p_W[k].transpose()    << "\n";
    std::cout << "v_W[k]    = " << v_W[k].transpose()    << "\n";
    std::cout << "a_W[k]    = " << a_W[k].transpose()    << "\n";
    std::cout << "R_WB[k] =\n" << R_WB[k]                << "\n";
    std::cout << "omega_B[k] = " << omega_B[k].transpose() << "\n";

    std::vector<Eigen::Vector3d> gyro(N), acc(N);
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d n_g(nd_g(rng), nd_g(rng), nd_g(rng));  // 每次掷骰子采 3 个值
        Eigen::Vector3d n_a(nd_a(rng), nd_a(rng), nd_a(rng));

        gyro[i] = omega_B[i] + b_g + n_g;          // 测量模型：角速度 + 偏置 + 噪声
        acc[i]  = R_WB[i] * f_W[i] + b_a + n_a;    // 比力搬到机体系 + 偏置 + 噪声
    }

    // ---- 验证 IMU 读数 ----
    std::cout << "gyro[k] = " << gyro[k].transpose() << "\n";
    std::cout << "acc[k]  = " << acc[k].transpose()  << "\n";
    // ① 定义变量：out 是"写往 imu_sim.bin 这个文件"的管道
    std::ofstream out(std::string(DATA_DIR) + "/imu_sim.bin", std::ios::binary);
    // ② 把数据写进管道（按 N,dt,然后所有 gyro/acc 逐时刻）
    //reinterpret_cast是用来强制二进制转换，把一个变量的内存地址，按照另一种类型来解释
    out.write(reinterpret_cast<const char*>(&N), sizeof(N));   // 先写 N
    out.write(reinterpret_cast<const char*>(&dt), sizeof(dt)); // 再写 dt
    for (int i = 0; i < N; ++i) {
        out.write(reinterpret_cast<const char*>(&gyro[i]), sizeof(gyro[i])); // 每个 gyro
        out.write(reinterpret_cast<const char*>(&acc[i]),  sizeof(acc[i]));  // 每个 acc
    }
    out.close();   // ③ 关管道（必须，否则数据可能没写完）
      return 0;
  }