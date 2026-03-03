# footstep_planner_double_layer.cpp 详细分析报告

## 1. 程序概述

`footstep_planner_double_layer.cpp`是一个基于**双层优化架构**的双足机器人落脚点规划程序，实现了从路径生成到动力学优化的完整流程。

### 核心功能
- 生成平滑的5次多项式引导路径
- 基于运动学约束的上层落脚点规划
- 基于ALIP（Angular Linear Inverted Pendulum）模型的下层动力学优化
- 可视化规划结果

### 技术栈
- **CasADi**：符号计算与优化求解
- **Eigen**：线性代数运算
- **glog**：日志系统
- **matplotlibcpp**：结果可视化

## 2. 依赖库与头文件

```cpp
#include <casadi/casadi.hpp>       // CasADi符号计算与优化库
#include <Eigen/Core>              // Eigen核心功能
#include <Eigen/Dense>             // Eigen稠密矩阵
#include <Eigen/LU>                // Eigen LU分解
#include <Eigen/QR>                // Eigen QR分解
#include <unsupported/Eigen/MatrixFunctions> // Eigen矩阵函数（如矩阵指数）
#include <iostream>                // 标准输入输出
#include <vector>                  // 向量容器
#include <cmath>                   // 数学函数
#include <algorithm>               // 算法库（如std::max）
#include <glog/logging.h>          // Google日志库
#include <matplotlibcpp.h>         // matplotlib C++接口
#include <chrono>                  // 计时功能

// 宏定义
#define ALIP                       // 启用ALIP动力学模型
#define GOAL_Hard_Constraint       // 启用目标硬约束
#define GOAL_Hard_Constraint_alip  // 启用ALIP层目标硬约束

using namespace std;
namespace plt = matplotlibcpp;     // matplotlib命名空间别名
```

## 3. 辅助函数

### 3.1 多项式函数模板

```cpp
template <typename T>
T polynomial_func(const Eigen::Vector<double, 6>& param, T x) {
    // 计算5次多项式值：a0*x^5 + a1*x^4 + a2*x^3 + a3*x^2 + a4*x + a5
    return param(0) * pow(x, 5) + param(1) * pow(x, 4) + param(2) * pow(x, 3) + 
           param(3) * pow(x, 2) + param(4) * x + param(5);
}

// CasADi MX类型的pow函数特化
casadi::MX pow(casadi::MX x, int n) { return casadi::MX::pow(x, n); }
```

**功能**：计算5次多项式的值，支持double和casadi::MX类型输入

### 3.2 ALIP自主矩阵计算

```cpp
Eigen::Matrix4d get_autonomous_alip_matrix_A(double H_com, double mass, double g) {
    Eigen::Matrix4d A_c_autonomous;
    // ALIP动力学矩阵
    A_c_autonomous << 0, 0, 0, 1 / (mass * H_com),
                      0, 0, -1 / (mass * H_com), 0,
                      0, -mass * g, 0, 0,
                      mass * g, 0, 0, 0;
    return A_c_autonomous;
}
```

**功能**：计算ALIP模型的自主系统矩阵

### 3.3 ALIP状态转移矩阵（单脚支撑期）

```cpp
std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 4, 2>> get_alip_matrices_with_input(double H_com, double mass, double g, double T_ss_dt) {
    Eigen::Matrix4d A_c = get_autonomous_alip_matrix_A(H_com, mass, g);
    Eigen::Matrix<double, 4, 2> B_c;
    B_c << 0, 0, 0, 0, 1, 0, 0, 1; // 简化力矩输入矩阵

    Eigen::Matrix4d A_d = (A_c * T_ss_dt).exp(); // 离散化状态转移矩阵
    Eigen::Matrix<double, 4, 2> B_d;
    
    // 计算输入矩阵，处理奇异性
    if (std::abs(A_c.determinant()) > 1e-9) {
         B_d = A_c.inverse() * (A_d - Eigen::Matrix4d::Identity()) * B_c;
    } else {
         B_d = B_c * T_ss_dt; // 近似处理
    }
    return {A_d, B_d};
}
```

**功能**：计算单脚支撑期的离散化状态转移矩阵和输入矩阵

### 3.4 Reset Map矩阵计算（双脚支撑期）

```cpp
std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 4, 3>> get_alip_reset_map_matrices_detailed(double T_ds, double H_com, double mass, double g) {
    Eigen::Matrix4d A_c = get_autonomous_alip_matrix_A(H_com, mass, g);
    Eigen::Matrix4d Ar_ds = (A_c * T_ds).exp(); // 双脚支撑期状态转移矩阵

    // 坐标系切换矩阵
    Eigen::Matrix<double, 4, 3> P_map;
    P_map.setZero();
    P_map(0,0) = 1.0; P_map(1,1) = 1.0; // 仅x,y位置变换

    // 计算Reset Map矩阵
    Eigen::Matrix<double, 4, 3> B_cop;
    B_cop <<0,0,0,0,0,0,0,mass * g,0,-mass * g,0,0;
    Eigen::Matrix<double, 4, 3> B_ds = Ar_ds * A_c.inverse() * (A_c.inverse() * (Eigen::Matrix4d::Identity() - Ar_ds.inverse())/T_ds - Ar_ds.inverse()) * B_cop;
    Eigen::Matrix<double, 4, 3> B_r = B_ds + P_map;
    return {Ar_ds, B_r};
}
```

**功能**：计算双脚支撑期的状态转移矩阵和Reset Map矩阵

## 4. 主函数

### 4.1 初始化与日志配置

```cpp
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]); // 初始化glog
    google::InstallFailureSignalHandler();
    FLAGS_minloglevel = 0; // 日志级别：0(INFO)、1(WARNING)、2(ERROR)
    FLAGS_colorlogtostderr = true; // 彩色日志
    FLAGS_alsologtostderr = true; // 同时输出到文件和stderr

    auto start_time = std::chrono::high_resolution_clock::now(); // 计时开始
```

### 4.2 路径生成与轨迹参数

```cpp
// 生成5次多项式引导路径
Eigen::Vector<double, 6> polynomial_param;
polynomial_param << 0, 0, -0.015625, 0.09375, 0, 0; // 5次多项式系数

// 设定起点和终点
double x_goal = 4.0; // 目标x坐标
double y_goal = polynomial_func(polynomial_param, x_goal); // 目标y坐标
LOG(INFO) << "Goal position: (" << x_goal << ", " << y_goal << ")";

// 计算目标朝向角
double delta_x = 0.01;
double delta_y = polynomial_func(polynomial_param, x_goal + delta_x) - polynomial_func(polynomial_param, x_goal);
double yaw_goal = atan2(delta_y, delta_x); // 目标偏航角

// 定义目标质心状态
Eigen::Vector4d x_com_goal(x_goal, y_goal, 0, 0);
```

### 4.3 机器人参数与初始状态

```cpp
// 初始支撑脚设置
bool initial_left_support = true; // 初始支撑脚为左脚
Eigen::Vector3d p_start_support_foot;
if (initial_left_support) {
    p_start_support_foot = Eigen::Vector3d(0.0, 0.1, 0.0); // 初始左脚支撑位置
} else {
    p_start_support_foot = Eigen::Vector3d(0.0, -0.1, 0.0); // 初始右脚支撑位置
}
```

### 4.4 步数估计

```cpp
// 基于距离的步数估计
double line_distance = std::sqrt(std::pow(x_goal - p_start_support_foot(0), 2) + std::pow(y_goal - p_start_support_foot(1), 2));
LOG(INFO) << "line distance: " << line_distance;
size_t steps_1 = std::ceil(line_distance / 0.3) + 2; // 步长0.3m，+2安全余量
LOG(INFO) << "Determined number of steps based on distance: " << steps_1;

// 基于转角的步数估计
double start_x = 0.0;
double total_yaw_change = 0.0;
double sample_step = 0.1;
double prev_yaw = 0.0;

for (double x = start_x + sample_step; x <= x_goal; x += sample_step) {
    double y = polynomial_func(polynomial_param, x);
    double y_next = polynomial_func(polynomial_param, x + delta_x);
    double current_yaw = atan2(y_next - y, delta_x);
    double yaw_diff = current_yaw - prev_yaw;
    // 角度归一化到[-π, π]
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
    total_yaw_change += std::abs(yaw_diff);
    prev_yaw = current_yaw;
}

size_t steps_2 = std::ceil(total_yaw_change / (M_PI / 12)) + 2; // 每步最大转15度
LOG(INFO) << "Determined number of steps based on yaw change: " << steps_2;

// 最终步数：取最大值
size_t N = std::max(steps_1, steps_2);
LOG(INFO) << "Determined number of steps N: " << N;

if (N > 20) {
    LOG(WARNING) << "Planned steps N is large (" << N << "), may lead to high computation time.";
}
```

## 5. 上层优化：运动学约束

### 5.1 优化器设置

```cpp
casadi::Opti opti = casadi::Opti(); // 创建优化器实例

// 定义优化变量：3×N矩阵，存储N个落脚点的(x,y,yaw)
casadi::MX P = opti.variable(3, N);
```

### 5.2 代价函数构建

```cpp
casadi::MX J = 0; // 初始化代价函数

// 1. 转角均匀代价
double lambda_yaw_smooth = 300.0;
for (size_t i = 0; i < N - 1; ++i) {
    J += lambda_yaw_smooth * (P(2, i + 1) - P(2, i)) * (P(2, i + 1) - P(2, i));
}

// 2. 步态均匀性代价
double lambda_step_smooth = 30.0;
for (size_t i = 0; i < N - 1; i++) {
    casadi::MX P_current = P(casadi::Slice(), i);
    casadi::MX P_next = P(casadi::Slice(), i + 1);
    J += lambda_step_smooth * ((P_next(0) - P_current(0))^2 + (P_next(1) - P_current(1))^2);
}

// 3. 转向角跟随路径代价
double lambda_yaw_guide = 5.0;
for (size_t i = 0; i < N; ++i) {
    casadi::MX cx = P(0, i);
    // 计算当前x处的切线斜率（多项式导数）
    casadi::MX dy_dx = 5*polynomial_param(0)*casadi::MX::pow(cx,4) + 4*polynomial_param(1)*casadi::MX::pow(cx,3) + 
                      3*polynomial_param(2)*casadi::MX::pow(cx,2) + 2*polynomial_param(3)*cx + polynomial_param(4);
    casadi::MX target_yaw = casadi::MX::atan(dy_dx);
    J += lambda_yaw_guide * (1.0 - casadi::MX::cos(P(2, i) - target_yaw));
}

// 4. 避免螃蟹步代价
double lambda_crab_avoid = 80.0;
for (size_t i = 0; i < N - 2; ++i) {
    casadi::MX delta_x = P(0, i + 2) - P(0, i);
    casadi::MX delta_y = P(1, i + 2) - P(1, i);
    casadi::MX yaw = P(2, i);
    // 计算横向位移
    casadi::MX lateral_disp = -casadi::MX::sin(yaw) * delta_x + casadi::MX::cos(yaw) * delta_y;
    J += lambda_crab_avoid * (lateral_disp * lateral_disp);
}

// 5. 路径跟踪代价
double lambda_path_tracking = 2.0;
for (size_t i = 1; i < N; ++i) {
    casadi::MX mid_x = (P(0, i - 1) + P(0, i)) / 2.0;
    casadi::MX mid_y = (P(1, i - 1) + P(1, i)) / 2.0;
    casadi::MX poly_y = polynomial_func(polynomial_param, mid_x);
    J += lambda_path_tracking * casadi::MX::sumsqr(poly_y - mid_y);
}

// 6. 终点方向角跟踪代价
double lambda_goal_yaw_tracking = 500.0;
J += lambda_goal_yaw_tracking * (1.0 - casadi::MX::cos(P(2, N - 1) - yaw_goal));

// 设置目标函数
opti.minimize(J);
```

### 5.3 约束条件

```cpp
// 1. 初始落脚点约束
casadi::MX P_initial = P(casadi::Slice(), 0);
opti.subject_to(P_initial(0) == p_start_support_foot(0));
opti.subject_to(P_initial(1) == p_start_support_foot(1));
opti.subject_to(P_initial(2) == 0); // 初始朝向为0

// 2. 终点硬约束（可选）
#ifdef GOAL_Hard_Constraint
    casadi::MX P_final = P(casadi::Slice(), N - 1); // 最后一步落脚点
    casadi::MX P_prev = P(casadi::Slice(), N - 2); // 倒数第二步落脚点

    // 约束两脚的朝向都必须对齐目标Yaw
    opti.subject_to(P_final(2) == yaw_goal);
    opti.subject_to(P_prev(2) == yaw_goal);

    casadi::MX dx_feet = P_final(0) - P_prev(0);
    casadi::MX dy_feet = P_final(1) - P_prev(1);

    // 3. 约束中点位置 = 目标点
    opti.subject_to((P_final(0) + P_prev(0)) / 2.0 == x_goal);
    opti.subject_to((P_final(1) + P_prev(1)) / 2.0 == y_goal);

    // 4. 约束连线方向垂直于目标朝向
    double cg = std::cos(yaw_goal);
    double sg = std::sin(yaw_goal);
    opti.subject_to(dx_feet * cg + dy_feet * sg == 0);

    // 5. 最小站立宽度约束
    double min_stance_width = 0.15; // 15cm
    casadi::MX dist_sq_feet = dx_feet * dx_feet + dy_feet * dy_feet;
    opti.subject_to(dist_sq_feet >= min_stance_width * min_stance_width);
    double max_stance_width = 0.30;
    opti.subject_to(dist_sq_feet <= max_stance_width * max_stance_width);
#endif

// 6. 相邻步转角约束
double max_step_yaw = M_PI / 12; // 15度
for (size_t i = 0; i < N - 1; ++i) {
    casadi::MX P_current = P(casadi::Slice(), i);
    casadi::MX P_next = P(casadi::Slice(), i + 1);
    opti.subject_to(casadi::MX::abs(P_next(2) - P_current(2)) <= max_step_yaw);
}

// 7. 双圆运动学约束
for (size_t i = 1; i < N; ++i) { // i为摆动脚编号
    bool cuurent_left_support;
    if (i % 2 == 0) cuurent_left_support = !initial_left_support;
    else cuurent_left_support = initial_left_support;

    casadi::MX support_center = casadi::MX::vertcat({P(0, i - 1), P(1, i - 1), 0});
    casadi::MX yaw_support = P(2, i - 1);
    
    double deta1 = 1.8; // 内圆圆心偏移量
    double deta2 = 0.35; // 外圆圆心偏移量
    double dis_th1 = 1.68; // 内圆半径
    double dis_th2 = 0.675; // 外圆半径

    casadi::MX P_next_pos = P(casadi::Slice(), i);
    casadi::MX sin_yaw = casadi::MX::sin(yaw_support);
    casadi::MX cos_yaw = casadi::MX::cos(yaw_support);

    if (!cuurent_left_support) { // 右脚支撑
        // 内圆约束
        casadi::MX p1_x = support_center(0) - deta1 * sin_yaw;
        casadi::MX p1_y = support_center(1) + deta1 * cos_yaw;
        casadi::MX dist1_sq = casadi::MX::pow(P_next_pos(0) - p1_x, 2) + casadi::MX::pow(P_next_pos(1) - p1_y, 2);
        opti.subject_to(dist1_sq <= dis_th1 * dis_th1);
        
        // 外圆约束
        casadi::MX p2_x = support_center(0) - (-deta2) * sin_yaw;
        casadi::MX p2_y = support_center(1) + (-deta2) * cos_yaw;
        casadi::MX dist2_sq = casadi::MX::pow(P_next_pos(0) - p2_x, 2) + casadi::MX::pow(P_next_pos(1) - p2_y, 2);
        opti.subject_to(dist2_sq <= dis_th2 * dis_th2);
    } else { // 左脚支撑
        // 内圆约束
        casadi::MX p1_x = support_center(0) - (-deta1) * sin_yaw;
        casadi::MX p1_y = support_center(1) + (-deta1) * cos_yaw;
        casadi::MX dist1_sq = casadi::MX::pow(P_next_pos(0) - p1_x, 2) + casadi::MX::pow(P_next_pos(1) - p1_y, 2);
        opti.subject_to(dist1_sq <= dis_th1 * dis_th1);
        
        // 外圆约束
        casadi::MX p2_x = support_center(0) - deta2 * sin_yaw;
        casadi::MX p2_y = support_center(1) + deta2 * cos_yaw;
        casadi::MX dist2_sq = casadi::MX::pow(P_next_pos(0) - p2_x, 2) + casadi::MX::pow(P_next_pos(1) - p2_y, 2);
        opti.subject_to(dist2_sq <= dis_th2 * dis_th2);
    }
}
```

### 5.4 求解上层优化

```cpp
// 设置求解器
opti.solver("ipopt");

// 添加初始猜测
for (int i = 0; i < N - 1; ++i) {
    double guess_px = p_start_support_foot(0) + i * 0.25; // 每步0.25m
    double guess_py = polynomial_func(polynomial_param, guess_px);
    double delta_x = 0.01;
    double delta_y = polynomial_func(polynomial_param, guess_px + delta_x) - polynomial_func(polynomial_param, guess_px);
    double guess_yaw = atan2(delta_y, delta_x);

    bool cuurent_left_support;
    if (i % 2 == 0) cuurent_left_support = !initial_left_support;
    else cuurent_left_support = initial_left_support;
    double foot_offset = 0.1;
    if (cuurent_left_support) {
        guess_px -= foot_offset * sin(guess_yaw);
        guess_py += foot_offset * cos(guess_yaw);
    } else {
        guess_px += foot_offset * sin(guess_yaw);
        guess_py -= foot_offset * cos(guess_yaw);
    }
    // 设置初始猜测值
    opti.set_initial(P(0, i), guess_px);
    opti.set_initial(P(1, i), guess_py);
    opti.set_initial(P(2, i), guess_yaw);
}

// 最后一步猜测
opti.set_initial(P(0, N - 1), x_goal);
opti.set_initial(P(1, N - 1), y_goal);
opti.set_initial(P(2, N - 1), yaw_goal);

// 求解优化问题
std::vector<double> res_px, res_py, res_yaw;
int time_cost = 0;
try {
    casadi::OptiSol sol = opti.solve();
    std::cout << "Optimization Success!" << std::endl;
    
    // 获取结果
    res_px = std::vector<double>(sol.value(P(0, casadi::Slice())));
    res_py = std::vector<double>(sol.value(P(1, casadi::Slice())));
    res_yaw = std::vector<double>(sol.value(P(2, casadi::Slice())));

    // 打印结果
    for(size_t i=0; i<res_px.size(); ++i) {
        std::cout << "Step " << i << ": (" << res_px[i] << ", " << res_py[i] << ", " << res_yaw[i] << ")" << std::endl;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Optimization Time: " << duration << " ms" << std::endl;
    time_cost = duration;

} catch(std::exception& e) {
    std::cerr << "Optimization Failed: " << e.what() << std::endl;
}
```

## 6. 下层优化：ALIP动力学

### 6.1 ALIP优化设置

```cpp
#ifdef ALIP
    auto start_time_alip = std::chrono::high_resolution_clock::now();
    
    // 动力学参数
    size_t k = 10; // 单脚支撑期离散化段数
    double H_com = 0.9, mass = 60, g = 9.81;
    double swing_t = 0.8, double_support = 0.2;
    double T_ss_dt = swing_t / k;

    // 获取ALIP矩阵
    auto [A_d_mpc, B_d_mpc_vec] = get_alip_matrices_with_input(H_com, mass, g, T_ss_dt);
    auto [Ar_reset, Br_reset_delta_p] = get_alip_reset_map_matrices_detailed(double_support, H_com, mass, g);
    
    // Eigen转CasADi DM
    casadi::DM A_d_mpc_dm = casadi::DM::zeros(4, 4);
    casadi::DM B_d_mpc_dm = casadi::DM::zeros(4, 2);
    casadi::DM Ar_reset_dm = casadi::DM::zeros(4, 4);
    casadi::DM Br_reset_dm = casadi::DM::zeros(4, 3);
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) A_d_mpc_dm(i, j) = A_d_mpc(i, j);
        for (int j = 0; j < 2; ++j) B_d_mpc_dm(i, j) = B_d_mpc_vec(i, j);
        for (int j = 0; j < 4; ++j) Ar_reset_dm(i, j) = Ar_reset(i, j);
        for (int j = 0; j < 3; ++j) Br_reset_dm(i, j) = Br_reset_delta_p(i, j);
    }
    
    // 创建ALIP优化器
    casadi::Opti opti_alip = casadi::Opti();
    
    // 定义优化变量
    casadi::MX X = opti_alip.variable(4, N * (k + 1)); // 状态变量
    casadi::MX X_goal = opti_alip.variable(4); // 终点状态
    casadi::MX P_alip = opti_alip.variable(3, N); // 落脚点变量
    casadi::MX U = opti_alip.variable(2, N * k); // 控制输入变量
```

### 6.2 ALIP代价函数

```cpp
    casadi::MX J_alip = 0;

    // 1. 转角均匀代价
double lambda_yaw_smooth_alip = 300.0;
for (size_t i = 0; i < N - 1; ++i) {
    J_alip += lambda_yaw_smooth_alip * (P_alip(2, i + 1) - P_alip(2, i)) * (P_alip(2, i + 1) - P_alip(2, i));
}

// 2. 步态均匀性代价
double lambda_step_smooth_alip = 13.0;
for (size_t i = 0; i < N - 1; i++) {
    casadi::MX P_current = P_alip(casadi::Slice(), i);
    casadi::MX P_next = P_alip(casadi::Slice(), i + 1);
    J_alip += lambda_step_smooth_alip * ((P_next(0) - P_current(0))^2 + (P_next(1) - P_current(1))^2);
}

// 3. 避免螃蟹步代价
double lambda_crab_avoid_alip = 80.0;
for (size_t i = 0; i < N - 2; ++i) {
    casadi::MX delta_x = P_alip(0, i + 2) - P_alip(0, i);
    casadi::MX delta_y = P_alip(1, i + 2) - P_alip(1, i);
    casadi::MX yaw = P_alip(2, i);
    casadi::MX lateral_disp = -casadi::MX::sin(yaw) * delta_x + casadi::MX::cos(yaw) * delta_y;
    J_alip += lambda_crab_avoid_alip * (lateral_disp * lateral_disp);
}

// 4. 控制输入最小化代价（避免主动力矩）
double lambda_u = 0.6;
for (size_t i = 0; i < N * k; ++i) {
    J_alip += lambda_u * (U(0, i) * U(0, i) + U(1, i) * U(1, i));
}

// 5. 终点约束代价
double lambda_goal_yaw = 500.0;
casadi::MX P_final_com = P_alip(casadi::Slice(), N - 1);
J_alip += lambda_goal_yaw * ((X_goal(0) + P_final_com(0) - x_com_goal(0))^2 + (X_goal(1) + P_final_com(1) - x_com_goal(1))^2);

// 6. 末端动量惩罚（刹车）
double lambda_terminal_state = 1000.0;
J_alip += lambda_terminal_state * (casadi::MX::pow(X_goal(2), 2) + casadi::MX::pow(X_goal(3), 2));

// 7. 落脚点跟踪代价（跟随上层规划）
double lambda_foot_tracking = 40.0;
for (size_t i = 0; i < N; ++i) {
    double ref_x = res_px[i];
    double ref_y = res_py[i];
    double ref_yaw = res_yaw[i];
    casadi::MX P_alip_current = P_alip(casadi::Slice(), i);
    J_alip += lambda_foot_tracking * ((ref_x - P_alip_current(0))^2 + (ref_y - P_alip_current(1))^2);
    J_alip += lambda_foot_tracking * (1.0 - casadi::MX::cos(ref_yaw - P_alip_current(2)));
}

// 设置目标函数
opti_alip.minimize(J_alip);
```

### 6.3 ALIP约束条件

```cpp
    // 1. 动力学约束
for (size_t i = 0; i < N; i++) { // 对每个步态周期
    // 单脚支撑期约束
    for (size_t j = 0; j < k; j++) { // 单脚支撑期的每个阶段
        size_t current_idx = i * (k + 1) + j;
        size_t next_idx = i * (k + 1) + j + 1;
        size_t control_idx = i * k + j;
        
        casadi::MX X_current = X(casadi::Slice(), current_idx);
        casadi::MX X_next = X(casadi::Slice(), next_idx);
        casadi::MX U_current = U(casadi::Slice(), control_idx);
        
        // 状态转移方程：X_next = A_d * X_current + B_d * U_current
        opti_alip.subject_to(X_next == casadi::MX::mtimes(A_d_mpc_dm, X_current) + casadi::MX::mtimes(B_d_mpc_dm, U_current));
    }
    
    // Reset阶段约束（双脚支撑期）
    if (i < N) {
        size_t current_idx = i * (k + 1) + k; // 单脚支撑末态
        casadi::MX X_current = X(casadi::Slice(), current_idx);
        casadi::MX X_next_state;
        if (i < N - 1) {
            size_t next_idx = (i + 1) * (k + 1);
            X_next_state = X(casadi::Slice(), next_idx);
        } else {
            X_next_state = X_goal; // 最后一个连接到目标状态
        }
        
        // 获取支撑脚位置
        casadi::MX P_current;
        if (i == 0) {
            P_current = casadi::MX::vertcat({p_start_support_foot(0), p_start_support_foot(1), 0});
        } else {
            P_current = casadi::MX::vertcat({P_alip(0, i - 1), P_alip(1, i - 1), 0});
        }
        casadi::MX P_next_step = casadi::MX::vertcat({P_alip(0, i), P_alip(1, i), 0});
        
        // Reset方程：X_next = Ar * X_curr + Br * (P_new - P_old)
        opti_alip.subject_to(X_next_state == casadi::MX::mtimes(Ar_reset_dm, X_current) + casadi::MX::mtimes(Br_reset_dm, P_next_step - P_current));
    }
}

// 2. 初始状态约束
casadi::MX X_initial = X(casadi::Slice(), 0);
double v_start_expected = 0.3; // 0.3 m/s启动速度
double Ly_start = mass * H_com * v_start_expected;

opti_alip.subject_to(X_initial(0) == 0);
opti_alip.subject_to(X_initial(1) == -0.01); // y轴相对位置
opti_alip.subject_to(X_initial(2) == 0); // Lx初始为0
opti_alip.subject_to(X_initial(3) == 0.5); // 给定初速度

// 3. 初始落脚点约束
casadi::MX P_initial_alip = P_alip(casadi::Slice(), 0);
opti_alip.subject_to(P_initial_alip(0) == p_start_support_foot(0));
opti_alip.subject_to(P_initial_alip(1) == p_start_support_foot(1));
opti_alip.subject_to(P_initial_alip(2) == 0);

// 4. 控制输入约束
double max_roll = 50.0;
double max_pitch = 50.0;
for (size_t i = 0; i < N * k; ++i) {
    casadi::MX U_current = U(casadi::Slice(), i);
    double limit_sq = max_roll * max_roll + max_pitch * max_pitch;
    opti_alip.subject_to(U_current(0)*U_current(0) + U_current(1)*U_current(1) <= limit_sq);
}

// 5. 终点硬约束（可选）
#ifdef GOAL_Hard_Constraint_alip
    // 与上层优化类似的终点约束
    // ...
#endif

// 6. 相邻步转角约束
double max_step_yaw_alip = M_PI / 12;
for (size_t i = 0; i < N - 1; ++i) {
    casadi::MX P_current = P_alip(casadi::Slice(), i);
    casadi::MX P_next = P_alip(casadi::Slice(), i + 1);
    opti_alip.subject_to(casadi::MX::abs(P_next(2) - P_current(2)) <= max_step_yaw_alip);
}

// 7. 双圆运动学约束（与上层优化类似）
// ...
```

### 6.4 求解ALIP优化

```cpp
    // 设置初始猜测
opti_alip.set_initial(U, 0.0); // 控制输入初始为0

// 使用上层规划结果作为落脚点初始猜测
for (int i = 0; i < N; ++i) {
    opti_alip.set_initial(P_alip(0, i), res_px[i]);
    opti_alip.set_initial(P_alip(1, i), res_py[i]);
    opti_alip.set_initial(P_alip(2, i), res_yaw[i]);
}

// 求解ALIP优化问题
try {
    casadi::OptiSol sol = opti_alip.solve();
    std::cout << "ALIP Optimization Success!" << std::endl;
    
    // 获取结果
    res_px = std::vector<double>(sol.value(P_alip(0, casadi::Slice())));
    res_py = std::vector<double>(sol.value(P_alip(1, casadi::Slice())));
    res_yaw = std::vector<double>(sol.value(P_alip(2, casadi::Slice())));

    // 打印结果
    for(size_t i=0; i<res_px.size(); ++i) {
        std::cout << "Step " << i << ": (" << res_px[i] << ", " << res_py[i] << ", " << res_yaw[i] << ")" << std::endl;
        Eigen::Vector2d sum_u = Eigen::Vector2d::Zero();
        double u_max = 0.0, v_max = 0.0;
        for (size_t j = 0; j < k; j++) {
            Eigen::Vector2d current_u(sol.value(U(0, j + k * i)), sol.value(U(1, j + k * i)));
            u_max = std::max(u_max, std::abs(current_u(0)));
            v_max = std::max(v_max, std::abs(current_u(1)));
            sum_u += current_u;
        }
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  Average Control U: (" << sum_u(0)/k << ", " << sum_u(1)/k << ")" << std::endl;
        std::cout << "  Max Control U: (" << u_max << ", " << v_max << ")" << std::endl;
    }
    auto end_time_alip = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_alip - start_time_alip).count();
    std::cout << "ALIP Optimization Time: " << duration << " ms" << std::endl;
    time_cost += duration;

} catch(std::exception& e) {
    std::cerr << "ALIP Optimization Failed: " << e.what() << std::endl;
}
#endif
```

## 7. 可视化部分

```cpp
// 可视化准备
std::vector<double> ref_x, ref_y;
for(double x = 0; x <= x_goal + 0.5; x += 0.05) {
    ref_x.push_back(x);
    ref_y.push_back(polynomial_func(polynomial_param, x));
}

// 轨迹连线数据
std::vector<double> traj_x, traj_y;
traj_x.push_back(p_start_support_foot(0));
traj_y.push_back(p_start_support_foot(1));
for(size_t i=0; i<res_px.size(); ++i) {
    traj_x.push_back(res_px[i]);
    traj_y.push_back(res_py[i]);
}

// 脚印分类与箭头数据
std::vector<double> left_foot_x, left_foot_y, left_u, left_v;
std::vector<double> right_foot_x, right_foot_y, right_u, right_v;

double arrow_len = 0.1;

// 处理起始脚
left_foot_x.push_back(p_start_support_foot(0));
left_foot_y.push_back(p_start_support_foot(1));
left_u.push_back(arrow_len * cos(p_start_support_foot(2)));
left_v.push_back(arrow_len * sin(p_start_support_foot(2)));

// 处理其他脚印
for(size_t i=0; i<res_px.size(); ++i) {
    double theta = res_yaw[i];
    double u_comp = arrow_len * cos(theta);
    double v_comp = arrow_len * sin(theta);

    if (i % 2 == 0) { // 右脚
        right_foot_x.push_back(res_px[i]);
        right_foot_y.push_back(res_py[i]);
        right_u.push_back(u_comp);
        right_v.push_back(v_comp);
    } else { // 左脚
        left_foot_x.push_back(res_px[i]);
        left_foot_y.push_back(res_py[i]);
        left_u.push_back(u_comp);
        left_v.push_back(v_comp);
    }
}

// 绘图
plt::figure_size(1200, 800);

// 画引导线
plt::plot(ref_x, ref_y, "k--");

// 画轨迹连线
plt::plot(traj_x, traj_y, "gray");

// 画左脚
plt::scatter(left_foot_x, left_foot_y, 50.0, {{"color", "red"}, {"label", "Left Foot"}});
plt::quiver(left_foot_x, left_foot_y, left_u, left_v, {{"color", "red"}});

// 画右脚
plt::scatter(right_foot_x, right_foot_y, 50.0, {{"color", "blue"}, {"label", "Right Foot"}});
plt::quiver(right_foot_x, right_foot_y, right_u, right_v, {{"color", "blue"}});

// 画目标点
std::vector<double> goal_pt_x = {x_goal};
std::vector<double> goal_pt_y = {y_goal};
plt::scatter(goal_pt_x, goal_pt_y, 100.0, {{"color", "green"}, {"marker", "*"}, {"label", "Goal"}});

// 设置图形属性
plt::title("Footstep Planning Result with ALIP " + std::to_string(N) + " Steps" + " duration: " + std::to_string(time_cost) + " ms");
plt::xlabel("X [m]");
plt::ylabel("Y [m]");
plt::axis("equal");
plt::legend();
plt::grid(true);

// 保存并显示
plt::save("footstep_plan.png");
plt::show();

return 0;
}
```

## 8. 调用流程

```
┌─────────────────────────────────────────────────────────┐
│                       程序启动                         │
├─────────────────────────────────────────────────────────┤
│                     初始化与配置                       │
│  - 日志配置                                            │
│  - 计时开始                                            │
├─────────────────────────────────────────────────────────┤
│                     路径与轨迹生成                     │
│  - 5次多项式引导路径                                   │
│  - 目标点与朝向计算                                    │
├─────────────────────────────────────────────────────────┤
│                     初始状态设置                       │
│  - 初始支撑脚位置                                      │
│  - 机器人参数初始化                                    │
├─────────────────────────────────────────────────────────┤
│                     步数估计                           │
│  - 基于距离的步数估计                                  │
│  - 基于转角的步数估计                                  │
│  - 最终步数确定                                        │
├─────────────────────────────────────────────────────────┤
│                    上层优化（运动学）                  │
│  - 优化变量定义                                        │
│  - 代价函数构建                                        │
│  - 约束条件设置                                        │
│  - 初始猜测设置                                        │
│  - 优化求解                                            │
├─────────────────────────────────────────────────────────┤
│                    下层优化（动力学）                  │
│  - ALIP模型参数设置                                    │
│  - 状态与控制变量定义                                  │
│  - 动力学代价函数构建                                  │
│  - 动力学约束条件设置                                  │
│  - 初始猜测设置                                        │
│  - 优化求解                                            │
├─────────────────────────────────────────────────────────┤
│                     结果可视化                         │
│  - 引导线绘制                                          │
│  - 轨迹连线绘制                                        │
│  - 脚印与朝向绘制                                      │
│  - 目标点标记                                          │
│  - 图形保存与显示                                      │
└─────────────────────────────────────────────────────────┘
```

## 9. 输入输出

### 输入
- **程序参数**：无命令行参数
- **配置参数**：
  - 多项式路径系数
  - 机器人物理参数（质量、质心高度等）
  - 优化权重
  - 约束参数

### 输出
- **控制台输出**：
  - 日志信息
  - 优化结果（各步落脚点坐标与朝向）
  - 控制输入信息
  - 优化时间
- **文件输出**：
  - `footstep_plan.png`：规划结果可视化图片

## 10. 总结

`footstep_planner_double_layer.cpp`实现了一个完整的双足机器人落脚点规划系统，具有以下特点：

1. **双层优化架构**：上层运动学优化保证路径跟随和行走自然性，下层动力学优化确保行走稳定性

2. **先进的动力学模型**：采用ALIP模型描述机器人行走动力学，平衡了模型精度和计算效率

3. **丰富的约束条件**：
   - 双圆运动学约束保证落脚可行性
   - 步态均匀性约束提高行走自然度
   - 路径跟踪约束确保沿规划路径行走
   - 螃蟹步避免约束提高行走效率

4. **可视化结果**：直观展示规划结果，便于分析和调试

该程序为双足机器人的行走规划提供了一套完整的解决方案，可应用于实际机器人系统的步态规划。