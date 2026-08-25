#include <iostream>
#include <Eigen/Geometry>

// MSVC 不提供 M_PI（GCC/Clang 自带），这里手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 把一个 3 维小旋转向量 δθ 变成四元数（指数映射）
// 小旋转向量 = 轴方向 × 旋转角，AngleAxis(角度, 轴) 正好就是"绕轴转这个角"
Eigen::Quaterniond exp_map(const Eigen::Vector3d& dtheta) {
    double angle = dtheta.norm();          // 旋转角 = 向量长度
    if (angle < 1e-10) return Eigen::Quaterniond::Identity();  // 零旋转 → 不转
    Eigen::Vector3d axis = dtheta / angle; // 旋转轴 = 向量方向
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

int main() {
    // 大姿态：绕世界 z 轴转 45°（一个"很大"的旋转，不可当向量）
    Eigen::Quaterniond q_big(Eigen::AngleAxisd(45.0 * M_PI / 180.0,
                                               Eigen::Vector3d::UnitZ()));
    // 小误差：绕 x 轴转 2°（很小，可当向量）
    Eigen::Vector3d dtheta(2.0 * M_PI / 180.0, 0.0, 0.0);

    // ① 大姿态被小误差"叠"上去
    Eigen::Quaterniond q_after = q_big * exp_map(dtheta);

    // ② 顺序反过来（先小误差、再大姿态）—— 小误差可交换，结果应几乎一样
    Eigen::Quaterniond q_rev = exp_map(dtheta) * q_big;

    // ③ 指数映射的近似式 [δθ/2, 1] 和精确四元数差多少
    //    注意：Quaterniond 构造函数参数是 (w, x, y, z)，w 在第一位！
    Eigen::Quaterniond q_approx(1.0, dtheta.x()/2.0, dtheta.y()/2.0,
                                dtheta.z()/2.0);
    q_approx.normalize();   // 让 |q|=1（近似式的 w≈1，需要归一化）

    // 用"车头向量"看效果
    Eigen::Vector3d v(1.0, 0.0, 0.0);
    std::cout << "原始车头                    : " << v.transpose() << "\n";
    std::cout << "大姿态转 45° 后             : " << (q_big * v).transpose() << "\n";
    std::cout << "大姿态 ⊗ exp(小误差) 后     : " << (q_after * v).transpose() << "\n";
    std::cout << "exp(小误差) ⊗ 大姿态 后     : " << (q_rev * v).transpose() << "\n";
    std::cout << "顺序反过来 vs 正序 差       : " << (q_after*v - q_rev*v).norm() << "\n\n";

    // ④ 验证"近似 vs 精确"
    std::cout << "exp 近似 [δθ/2,1] 的车头    : " << (q_approx * v).transpose() << "\n";
    std::cout << "精确 exp(δθ) 的车头         : " << (exp_map(dtheta) * v).transpose() << "\n";
    std::cout << "近似 vs 精确 差             : " << (q_approx*v - exp_map(dtheta)*v).norm() << "\n";
    return 0;
}
