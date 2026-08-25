#include <iostream>
#include <random>
#include <iomanip>

int main() {
    // ================= 场景设定 =================
    const double x_true = 10.0;   // 真值：物体静止在 10 米（只有上帝视角知道）
    const double R = 1.0;         // 测量方差：传感器多吵（噪声 σ=1）
    const double Q = 0.01;        // 过程噪声：物体静止，模型很准，Q 取很小

    // 高斯"骰子"：每次掷出一个均值为0、方差R的噪声
    std::mt19937 rng(42);
    std::normal_distribution<double> nd(0.0, std::sqrt(R));

    // ================= 滤波器状态 =================
    double x_hat = 0.0;    // 估计位置：先猜它在 0 米
    double P     = 100.0;  // 把握度：完全没谱 → 方差巨大

    const int N = 50;      // 观测 50 次
    std::cout << "  拍   测量 z   估计 x̂     方差 P     增益 K\n";

    for (int i = 1; i <= N; ++i) {
        // ① 拿到一次观测：真值 + 噪声
        double z = x_true + nd(rng);

        // ② 预测：模型说"它还在这"，但世界在动，P 涨一点
        P += Q;

        // ③ 算增益：这拍该信谁
        double K = P / (P + R);

        // ④ 用新息 (z - x̂) 把估计往测量方向拉 K 比例
        x_hat += K * (z - x_hat);

        // ⑤ 注入了一条信息，P 收缩
        P *= (1.0 - K);

        // 只打印前 10 拍和后 5 拍
        if (i <= 10 || i > N - 5) {
            std::cout << std::setw(4) << i
                      << std::fixed << std::setprecision(2)
                      << std::setw(10) << z
                      << std::setw(10) << x_hat
                      << std::setw(10) << P
                      << std::setw(10) << K << "\n";
        }
    }
    return 0;
}
