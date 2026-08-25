// ============================================================
//  阶段6a：ESKF 在 EuRoC 真实数据上运行
//
//  与阶段4 的区别（都是"真实数据"逼出来的）：
//    ① 数据源：EuRoC CSV（带纳秒时间戳），不再是无时间戳的二进制流
//    ② 每步 dt 用真实时间戳差值（IMU 不是严格均匀采样）
//    ③ 观测调度：按时间戳驱动（GT 每 0.2s 给一个位置观测），
//       绝不按"第几个索引"对齐 —— 两个传感器各有各的时钟
//    ④ 噪声参数：从 sensor.yaml 读噪声密度（σ := density，见笔记推导）
//    ⑤ 姿态/重力符号做自检（静态 acc z 应 ≈ +9.81）
//
//  注意（诚实声明）：EuRoC 没有位置传感器，只有 IMU + 相机。
//  这里用"真值位置 + 小噪声"充当位置观测替身（和阶段4 用
//  解析真值当雷达同构），评价时和全速率真值对比 —— 所以
//  "两次观测之间 IMU 的传播质量"依然是有意义的。
//
//  用法：cmake 构建后直接运行 build/stage6_euroc [数据目录]
//  默认数据目录 = data/euroc（含 imu0/ 和 state_groundtruth_estimate0/）
//  仿真测试：先 python3 scripts/gen_euroc_sim.py 生成 data/euroc_sim
//           （EuRoC 同款目录结构），再 build/stage6_euroc data/euroc_sim
// ============================================================
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <random>
#include <iomanip>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eskf.hpp"

// 每行文本按逗号切出若干字段
static std::vector<double> split_csv_line(const std::string& line) {
    std::vector<double> out;
    std::string token;
    std::istringstream ss(line);
    while (std::getline(ss, token, ',')) {
        // 表头行第一个 token 可能是 "#timestamp"，剥掉前导 '#' 再解析
        if (out.empty() && !token.empty() && token[0] == '#') token.erase(token.begin());
        // 跳过空 token（行尾多余逗号等）
        if (token.empty()) continue;
        try {
            out.push_back(std::stod(token));
        } catch (...) {
            out.clear();   // 表头/非数字行 → 返回空，调用方跳过
            return out;
        }
    }
    return out;
}

struct ImuSample { double t; Eigen::Vector3d w, a; };
struct GtSample  { double t; Eigen::Vector3d p, v, b_g, b_a; Eigen::Quaterniond q; };

// ---------- 读取 IMU CSV：timestamp, w_xyz(rad/s), a_xyz(m/s² 比力) ----------
static std::vector<ImuSample> load_imu(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::cerr << "无法打开 " << path << "\n"; exit(1); }
    std::vector<ImuSample> imu;
    std::string line;
    bool header_skipped = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        if (!header_skipped) { header_skipped = true; continue; }  // 第一行表头
        auto v = split_csv_line(line);
        if (v.size() < 7) continue;
        ImuSample s;
        s.t = v[0] * 1e-9;                       // ns → s
        s.w = Eigen::Vector3d(v[1], v[2], v[3]);
        s.a = Eigen::Vector3d(v[4], v[5], v[6]);
        imu.push_back(s);
    }
    // 时间戳必须单调递增（真实数据约定）；异常说明解析/数据有问题
    for (size_t i = 1; i < imu.size(); ++i)
        if (imu[i].t <= imu[i-1].t)
            std::cerr << "!! IMU 时间戳非递增 @ " << i << ": "
                      << imu[i-1].t << " -> " << imu[i].t << "\n";
    return imu;
}

// ---------- 读取真值 CSV：#timestamp, p_xyz, q_wxyz, v_xyz, b_g, b_a ----------
static std::vector<GtSample> load_gt(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::cerr << "无法打开 " << path << "\n"; exit(1); }
    std::vector<GtSample> gt;
    std::string line;
    bool header_skipped = false;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        if (!header_skipped) { header_skipped = true; continue; }
        auto v = split_csv_line(line);
        if (v.size() < 17) continue;
        GtSample s;
        s.t = v[0] * 1e-9;
        s.p = Eigen::Vector3d(v[1], v[2], v[3]);
        s.q = Eigen::Quaterniond(v[4], v[5], v[6], v[7]).normalized();  // wxyz
        s.v = Eigen::Vector3d(v[8], v[9], v[10]);
        s.b_g = Eigen::Vector3d(v[11], v[12], v[13]);   // 真值陀螺偏置
        s.b_a = Eigen::Vector3d(v[14], v[15], v[16]);   // 真值加速度计偏置
        gt.push_back(s);
    }
    for (size_t i = 1; i < gt.size(); ++i)
        if (gt[i].t <= gt[i-1].t)
            std::cerr << "!! GT 时间戳非递增 @ " << i << "\n";
    return gt;
}

// ---------- 读取 sensor.yaml 里的噪声密度 ----------
static double yaml_double(const std::string& path, const std::string& key, double fallback) {
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string k = line.substr(0, colon);
        k.erase(std::remove_if(k.begin(), k.end(), ::isspace), k.end());
        if (k == key) {
            try { return std::stod(line.substr(colon + 1)); }
            catch (...) { return fallback; }
        }
    }
    return fallback;
}

// ---------- 观测调度：GT 位置每 obs_interval 秒取一次，匹配最近 IMU 索引 ----------
static std::map<size_t, Eigen::Vector3d>
build_measurements(const std::vector<GtSample>& gt,
                   const std::vector<double>& imu_t,
                   double obs_interval, double sigma_obs, std::mt19937& rng) {
    std::map<size_t, Eigen::Vector3d> m;
    std::normal_distribution<double> nd(0.0, sigma_obs);
    double last = gt.empty() ? -1e9 : gt[0].t;   // 首样本已用于初始化，不再观测
    for (size_t j = 0; j < gt.size(); ++j) {
        if (gt[j].t - last < obs_interval) continue;
        size_t k = std::lower_bound(imu_t.begin(), imu_t.end(), gt[j].t) - imu_t.begin();
        if (k >= imu_t.size()) k = imu_t.size() - 1;
        if (k > 0 && std::abs(imu_t[k-1] - gt[j].t) < std::abs(imu_t[k] - gt[j].t)) --k;
        Eigen::Vector3d z = gt[j].p;
        for (int c = 0; c < 3; ++c) z[c] += nd(rng);   // 传感器替身的噪声
        m[k] = z;
        last = gt[j].t;
    }
    return m;
}

// ---------- 把 IMU 索引的轨迹按 GT 时间戳重采样（两种轨迹对齐到同一时间轴） ----------
static std::vector<Eigen::Vector3d>
sample_at_gt(const std::vector<Eigen::Vector3d>& traj,
             const std::vector<double>& imu_t, const std::vector<GtSample>& gt) {
    std::vector<Eigen::Vector3d> out(gt.size());
    for (size_t j = 0; j < gt.size(); ++j) {
        size_t k = std::lower_bound(imu_t.begin(), imu_t.end(), gt[j].t) - imu_t.begin();
        if (k >= imu_t.size()) k = imu_t.size() - 1;
        out[j] = traj[k];
    }
    return out;
}

// ---------- 指标 ----------
static double ate(const std::vector<Eigen::Vector3d>& est,
                  const std::vector<Eigen::Vector3d>& tru) {
    double s = 0.0;
    for (size_t i = 0; i < est.size(); ++i) s += (est[i] - tru[i]).squaredNorm();
    return std::sqrt(s / est.size());
}
static double rpe(const std::vector<Eigen::Vector3d>& est,
                  const std::vector<Eigen::Vector3d>& tru, size_t delta) {
    double s = 0.0; size_t m = 0;
    for (size_t i = 0; i + delta < est.size(); ++i) {
        s += ((est[i+delta] - est[i]) - (tru[i+delta] - tru[i])).squaredNorm();
        ++m;
    }
    return std::sqrt(s / m);
}

int main(int argc, char** argv) {
    // ---- 数据目录：argv[1] 可覆盖，默认 DATA_DIR/euroc ----
    std::string euroc = (argc > 1) ? argv[1] : std::string(DATA_DIR) + "/euroc";
    const std::string imu_csv = euroc + "/imu0/data.csv";
    const std::string gt_csv  = euroc + "/state_groundtruth_estimate0/data.csv";
    const std::string yaml    = euroc + "/imu0/sensor.yaml";

    // ---- 读数据 ----
    auto imu = load_imu(imu_csv);
    auto gt  = load_gt(gt_csv);
    std::cout << "IMU 样本数 " << imu.size() << "，GT 样本数 " << gt.size()
              << "，时长 " << (imu.back().t - imu.front().t) << " s\n";

    // ---- 噪声参数：sensor.yaml 优先，缺了用默认 ----
    eskf::NoiseParams np;
    np.sigma_g   = yaml_double(yaml, "gyroscope_noise_density",    np.sigma_g);
    np.sigma_a   = yaml_double(yaml, "accelerometer_noise_density",np.sigma_a);
    np.sigma_bg  = yaml_double(yaml, "gyroscope_random_walk",      np.sigma_bg);
    np.sigma_ba  = yaml_double(yaml, "accelerometer_random_walk",  np.sigma_ba);
    np.sigma_obs = 0.1;   // 位置观测替身的噪声 std (m)
    std::cout << "噪声参数 (来自 sensor.yaml): sigma_g=" << np.sigma_g
              << " sigma_a=" << np.sigma_a
              << " sigma_bg=" << np.sigma_bg << " sigma_ba=" << np.sigma_ba << "\n";

    // ---- 重力/坐标符号自检：静止且水平的 IMU，比力 z 应 ≈ +9.81 ----
    // 前提：V1_01_easy 开头无人机静止平放在桌上
    {
        int n = std::min<size_t>(200, imu.size());   // 前 1s（200Hz）
        double az = 0.0;
        for (int i = 0; i < n; ++i) az += imu[i].a.z();
        az /= n;
        std::cout << "前 1s 比力 z 均值 = " << az
                  << "  (静态水平应 ≈ +9.81；若 ≈ -9.81 说明重力符号反了)\n";
        if (std::abs(std::abs(az) - 9.81) > 0.5) {
            std::cerr << "!! 比力 z 不在 ±9.81 附近，开头可能非静止/非水平，结果需谨慎\n";
        }
    }
    std::cout << "GT 前 1s 位置 z: " << gt.front().p.z() << " -> "
              << gt[std::min<size_t>(100, gt.size()-1)].p.z() << " (应几乎不变)\n";

    // ---- 初始化：大状态来自 GT 首样本，偏置从 0 学起 ----
    eskf::State s0;
    s0.p = gt.front().p;
    s0.v = gt.front().v;
    s0.q = gt.front().q;
    eskf::Eskf eskf;
    eskf.reset(s0, np, 9.81);

    std::vector<double> imu_t(imu.size());
    for (size_t i = 0; i < imu.size(); ++i) imu_t[i] = imu[i].t;

    // ---- 观测调度 + 主循环 ----
    std::mt19937 rng(7);
    auto measures = build_measurements(gt, imu_t, 0.2, np.sigma_obs, rng);
    std::cout << "位置观测次数 = " << measures.size()
              << "（约 5Hz）\n";

    std::vector<Eigen::Vector3d> eskf_traj(imu.size()), pure_traj(imu.size());
    std::vector<Eigen::Vector3d> bg_hist(imu.size()), ba_hist(imu.size());
    eskf_traj[0] = s0.p;
    pure_traj[0] = s0.p;
    bg_hist[0] = s0.b_g; ba_hist[0] = s0.b_a;

    // 纯积分基线（不校正、不除偏置 —— 阶段1 的做法，观察真实漂移）
    Eigen::Vector3d pp = s0.p, vp = s0.v;
    Eigen::Quaterniond qp = s0.q;

    size_t n_obs = 0;
    for (size_t i = 1; i < imu.size(); ++i) {
        double dt = imu_t[i] - imu_t[i-1];
        if (dt <= 0) {            // 乱序/重复时间戳：跳过本步
            eskf_traj[i] = eskf_traj[i-1];
            pure_traj[i] = pure_traj[i-1];
            continue;
        }
        if (dt > 0.1) dt = 0.1;   // 大缺口封顶，防止协方差爆掉

        // === ESKF 预测 + 按需观测更新 ===
        eskf.predict(imu[i].w, imu[i].a, dt);
        auto it = measures.find(i);
        if (it != measures.end()) { eskf.update(it->second); ++n_obs; }
        eskf_traj[i] = eskf.state().p;
        bg_hist[i] = eskf.state().b_g;
        ba_hist[i] = eskf.state().b_a;

        // === 纯积分基线 ===
        Eigen::Vector3d aW = qp.toRotationMatrix() * imu[i].a
                             + Eigen::Vector3d(0, 0, -9.81);
        pp += vp * dt + 0.5 * aW * dt * dt;
        vp += aW * dt;
        qp = qp * eskf::exp_map(imu[i].w * dt);
        qp.normalize();
        pure_traj[i] = pp;
    }

    // ---- 按 GT 时间戳对齐后算 ATE/RPE ----
    auto tru_p = [&](const GtSample& g) { return g.p; };
    std::vector<Eigen::Vector3d> gt_p(gt.size());
    for (size_t j = 0; j < gt.size(); ++j) gt_p[j] = tru_p(gt[j]);

    auto eskf_at = sample_at_gt(eskf_traj, imu_t, gt);
    auto pure_at = sample_at_gt(pure_traj, imu_t, gt);

    size_t delta = 1;
    while (delta < gt.size() && gt[delta].t - gt[0].t < 1.0) ++delta;   // RPE 间隔 1s

    double ate_p = ate(pure_at, gt_p), ate_e = ate(eskf_at, gt_p);
    double rpe_p = rpe(pure_at, gt_p, delta), rpe_e = rpe(eskf_at, gt_p, delta);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n========== ATE / RPE (" << euroc << ") ==========\n";
    std::cout << "ATE (绝对轨迹误差) : 纯积分 " << std::setw(9) << ate_p
              << " m | ESKF " << std::setw(9) << ate_e
              << " m | 改善 " << ate_p / ate_e << " 倍\n";
    std::cout << "RPE (相对位姿误差) : 纯积分 " << std::setw(9) << rpe_p
              << " m | ESKF " << std::setw(9) << rpe_e
              << " m | 改善 " << rpe_p / rpe_e << " 倍\n";
    // 最大误差 = 到 GT 的最大距离（注意别写成离原点的距离，那没有意义）
    double max_e_p = 0, max_e_e = 0;
    for (size_t j = 0; j < gt.size(); ++j) {
        max_e_p = std::max(max_e_p, (pure_at[j] - gt_p[j]).norm());
        max_e_e = std::max(max_e_e, (eskf_at[j] - gt_p[j]).norm());
    }
    std::cout << "纯积分最大误差 : " << max_e_p
              << " m | ESKF 最大误差 : " << max_e_e << " m\n";
    std::cout << "最终陀螺偏置估计 : " << eskf.state().b_g.transpose()
              << "  加速度计偏置 : " << eskf.state().b_a.transpose() << "\n";
    std::cout << "(注：位置观测对偏航弱可观测，b_g.z 通常收敛很差，属正常)\n";

    // ---- 导出 CSV：轨迹 + 偏置历史（供画图） ----
    auto bg_at = sample_at_gt(bg_hist, imu_t, gt);
    auto ba_at = sample_at_gt(ba_hist, imu_t, gt);
    std::ofstream out(std::string(DATA_DIR) + "/euroc_traj.csv");
    out << "t,gt_x,gt_y,gt_z,pure_x,pure_y,pure_z,eskf_x,eskf_y,eskf_z,"
           "gt_bg_x,gt_bg_y,gt_bg_z,gt_ba_x,gt_ba_y,gt_ba_z,"
           "bg_x,bg_y,bg_z,ba_x,ba_y,ba_z\n";
    for (size_t j = 0; j < gt.size(); ++j) {
        out << std::fixed << std::setprecision(6)
            << gt[j].t << ","
            << gt_p[j].x() << "," << gt_p[j].y() << "," << gt_p[j].z() << ","
            << pure_at[j].x() << "," << pure_at[j].y() << "," << pure_at[j].z() << ","
            << eskf_at[j].x() << "," << eskf_at[j].y() << "," << eskf_at[j].z() << ","
            << gt[j].b_g.x() << "," << gt[j].b_g.y() << "," << gt[j].b_g.z() << ","
            << gt[j].b_a.x() << "," << gt[j].b_a.y() << "," << gt[j].b_a.z() << ","
            << bg_at[j].x() << "," << bg_at[j].y() << "," << bg_at[j].z() << ","
            << ba_at[j].x() << "," << ba_at[j].y() << "," << ba_at[j].z() << "\n";
    }
    out.close();
    std::cout << "已导出 data/euroc_traj.csv（轨迹对齐到 GT 时间轴）\n";
    std::cout << "画图：python3 scripts/stage6_plot.py\n";
    return 0;
}
