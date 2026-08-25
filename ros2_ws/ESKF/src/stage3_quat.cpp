#include <iostream>
#include <Eigen/Geometry>

int main() {
    const double pi = 3.14159265358979;

    // ① 用"角度 + 旋转轴"构造旋转（最符合直觉的写法）
    //    绕 z 轴转 30°：偏航（小车在水平面转头）
    //    绕 y 轴转 45°：俯仰（车头往天上仰）
    Eigen::AngleAxisd rot_z(30.0 * pi / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_y(45.0 * pi / 180.0, Eigen::Vector3d::UnitY());

    // ② 从"角度+轴"生成四元数
    Eigen::Quaterniond qz(rot_z);
    Eigen::Quaterniond qy(rot_y);
    std::cout << "qz = " << qz.coeffs().transpose()
              << "  (x,y,z,w)，w 是标量部分\n\n";

    // ③ 复合旋转：q_zy = qy * qz 表示"先转 qz，再转 qy"
    //    （右乘的是先发生的，和矩阵复合一个规矩）
    Eigen::Quaterniond q_zy = qy * qz;   // 先 z 后 y
    Eigen::Quaterniond q_yz = qz * qy;   // 先 y 后 z

    // ④ 关键演示：交换顺序结果不同（旋转不满足交换律）
    Eigen::Vector3d v(1.0, 0.0, 0.0);    // 车头朝向（世界系）
    Eigen::Vector3d v_zy = q_zy * v;     // 四元数作用在向量上 = 旋转它
    Eigen::Vector3d v_yz = q_yz * v;
    std::cout << "先转z后转y，车头指向: " << v_zy.transpose() << "\n";
    std::cout << "先转y后转z，车头指向: " << v_yz.transpose() << "\n";
    std::cout << "两者差 " << (v_zy - v_yz).norm()
              << "  —— 顺序不同，结果不同！\n\n";

    // ⑤ 四元数 ↔ 旋转矩阵：同一个旋转的两种写法
    Eigen::Matrix3d R = q_zy.toRotationMatrix();
    std::cout << "q_zy 对应的旋转矩阵 R =\n" << R << "\n";
    // 用矩阵旋转同一个向量，结果必须和四元数一样
    std::cout << "R * v = " << (R * v).transpose()
              << "  （和 q_zy*v 一模一样）\n\n";

    // ⑥ 从矩阵还原四元数，转一圈回来应该不变
    Eigen::Quaterniond q_back(R);
    std::cout << "矩阵→四元数 q_back = " << q_back.coeffs().transpose() << "\n";
    std::cout << "和 q_zy 的差 = " << (q_back.coeffs() - q_zy.coeffs()).norm()
              << "\n\n";

    // ⑦ 四元数永远单位长度（这就是它紧凑、不会漂的秘密）
    std::cout << "|q_zy| = " << q_zy.norm() << "  （永远是 1）\n";
    return 0;
}
