#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

int main() {
    // ---- 打开文件，二进制读入 ----
    std::ifstream in(std::string(DATA_DIR) + "/imu_sim.bin", std::ios::binary);
    if (!in) {
        std::cerr << "无法打开 imu_sim.bin\n";
        return 1;
    }

    // ---- 按写入顺序读回：N, dt, 然后 N 组 gyro/acc ----
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

    // ---- 验证：打印第 0 个和第 250 个时刻 ----
    for (int k : {0, 250}) {
        std::cout << "gyro[" << k << "] = " << gyro[k].transpose() << "\n";
        std::cout << "acc["  << k << "] = " << acc[k].transpose()  << "\n";
    }
    return 0;
}
