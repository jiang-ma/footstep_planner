#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <glog/logging.h>
#include <matplotlibcpp.h>
#include <chrono>

#include "../include/stair_geometry.h"
#include "../include/visualization_3d.h"

using namespace std;
namespace plt = matplotlibcpp;

// 多项式函数模板（用于初始路径生成）
template <typename T>
T polynomial_func(const Eigen::Vector<double, 6>& param, T x) {
    return param(0) * pow(x, 5) + param(1) * pow(x, 4) + param(2) * pow(x, 3) + 
           param(3) * pow(x, 2) + param(4) * x + param(5);
}

// CasADi MX版本的pow函数
casadi::MX pow(casadi::MX x, int n) { return casadi::MX::pow(x, n); }

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;

    auto start_time = std::chrono::high_resolution_clock::now();

    // ==========================================
    // 1. 楼梯几何定义（重新设计为多个台阶平面）
    // ==========================================
    
    // 楼梯参数
    double step_height = 0.15;  // 台阶高度
    double step_width = 0.3;    // 台阶宽度
    int step_count = 8;         // 台阶数量
    double stair_width = 1.0;   // 楼梯宽度
    
    // 创建8个台阶平面
    std::vector<std::vector<Eigen::Vector3d>> step_planes;
    std::vector<Eigen::Vector3d> step_centers;
    
    // 机器人位置
    Eigen::Vector3d robot_position(0.0, 0.0, 0.0);
    
    for (int i = 0; i < step_count; ++i) {
        // 计算台阶位置
        double x_start = 0.8 + i * step_width;  // 从0.8米开始
        double x_end = x_start + step_width;
        double z = (i + 1) * step_height;  // 台阶高度
        
        // 台阶中心点
        Eigen::Vector3d center((x_start + x_end) / 2, 0.0, z);
        step_centers.push_back(center);
        
        // 台阶多边形顶点（矩形）
        std::vector<Eigen::Vector3d> vertices = {
            Eigen::Vector3d(x_start, -stair_width/2, z),  // 左下
            Eigen::Vector3d(x_end, -stair_width/2, z),    // 右下
            Eigen::Vector3d(x_end, stair_width/2, z),     // 右上
            Eigen::Vector3d(x_start, stair_width/2, z)     // 左上
        };
        step_planes.push_back(vertices);
        
        LOG(INFO) << "Step " << i << ": x=[" << x_start << ", " << x_end 
                  << "], z=" << z << ", center=(" << center.x() << ", " 
                  << center.y() << ", " << center.z() << ")";
    }
    
    // 计算侵蚀后的安全区域（每个台阶向内收缩0.1米）
    std::vector<std::vector<Eigen::Vector3d>> eroded_step_planes;
    for (const auto& vertices : step_planes) {
        std::vector<Eigen::Vector3d> eroded_vertices;
        for (size_t j = 0; j < vertices.size(); ++j) {
            Eigen::Vector3d prev = vertices[(j + vertices.size() - 1) % vertices.size()];
            Eigen::Vector3d curr = vertices[j];
            Eigen::Vector3d next = vertices[(j + 1) % vertices.size()];
            
            // 计算侵蚀后的顶点（向内收缩0.1米）
            Eigen::Vector3d edge1 = (curr - prev).normalized();
            Eigen::Vector3d edge2 = (next - curr).normalized();
            Eigen::Vector3d normal1(-edge1.y(), edge1.x(), 0);
            Eigen::Vector3d normal2(-edge2.y(), edge2.x(), 0);
            
            Eigen::Vector3d eroded_point = curr + 0.1 * (normal1 + normal2).normalized();
            eroded_point.z() = curr.z();
            eroded_vertices.push_back(eroded_point);
        }
        eroded_step_planes.push_back(eroded_vertices);
    }
    
    // 根据距离排序台阶（离机器人最近的台阶排前面）
    std::vector<std::pair<double, int>> step_distances; // 距离, 台阶索引
    for (int i = 0; i < step_count; ++i) {
        double distance = (step_centers[i] - robot_position).norm();
        step_distances.push_back({distance, i});
    }
    std::sort(step_distances.begin(), step_distances.end());
    
    LOG(INFO) << "Stair geometry created: " << step_count << " step planes, height=" 
              << step_height << ", width=" << step_width;
    LOG(INFO) << "Step distances from robot:";
    for (const auto& [distance, index] : step_distances) {
        LOG(INFO) << "  Step " << index << ": distance=" << distance << "m";
    }

    // ==========================================
    // 2. 目标点定义
    // ==========================================
    
    // 计算楼梯实际长度和端点
    double total_stair_length = step_count * step_width;  // 8*0.3=2.4米
    double stair_start_x = 0.8;  // 第一个台阶起始位置
    double stair_end_x = stair_start_x + total_stair_length;  // 0.8 + 2.4 = 3.2米
    
    // 目标点设置在楼梯顶部
    double x_goal = stair_end_x + 0.5 * step_width;  // 楼梯末端再延伸半个台阶
    double y_goal = 0.0;  // 楼梯中心线
    
    // 计算目标朝向（沿楼梯中心线方向）
    double yaw_goal = 0.0;  // 沿x轴方向
    
    LOG(INFO) << "Goal position: (" << x_goal << ", " << y_goal << "), yaw: " << yaw_goal;

    // ==========================================
    // 3. 初始条件设置
    // ==========================================
    
    bool initial_left_support = true;
    Eigen::Vector3d p_start_support_foot;
    
    if (initial_left_support) {
        p_start_support_foot = Eigen::Vector3d(0.0, 0.1, 0.0); // 初始左脚支撑
    } else {
        p_start_support_foot = Eigen::Vector3d(0.0, -0.1, 0.0); // 初始右脚支撑
    }

    // ==========================================
    // 4. 步数估计
    // ==========================================
    
    // 基于距离估计步数
    double total_distance = sqrt(pow(x_goal - p_start_support_foot(0), 2) + 
                                pow(y_goal - p_start_support_foot(1), 2));
    
    // 楼梯区域内的步长较小，平地区域的步长较大
    double flat_step_length = 0.3;   // 平地步长
    double stair_step_length = 0.2;  // 楼梯步长
    
    // 估计楼梯区域内的步数
    double stair_length = step_count * step_width;
    double flat_length = total_distance - stair_length;
    
    size_t steps_flat = std::ceil(flat_length / flat_step_length);
    size_t steps_stair = std::ceil(stair_length / stair_step_length);
    size_t N = steps_flat + steps_stair + 2;  // 额外增加2步作为缓冲
    
    N = std::min(N, static_cast<size_t>(20));  // 限制最大步数
    
    LOG(INFO) << "Total distance: " << total_distance << ", Planned steps: " << N;
    LOG(INFO) << "Flat steps: " << steps_flat << ", Stair steps: " << steps_stair;

    // ==========================================
    // 5. 优化器设置
    // ==========================================
    
    casadi::Opti opti = casadi::Opti();

    // --- 变量定义 ---
    // 只优化x,y坐标，yaw由楼梯几何决定
    casadi::MX P_xy = opti.variable(2, N);  // x,y坐标
    casadi::MX P_yaw = opti.variable(1, N); // 朝向角

    // --- 代价函数 ---
    casadi::MX J = 0;

    // 主要目标：到达目标点
    double lambda_goal = 100.0;
    J += lambda_goal * ((P_xy(0, N - 1) - x_goal) * (P_xy(0, N - 1) - x_goal) + 
                        (P_xy(1, N - 1) - y_goal) * (P_xy(1, N - 1) - y_goal));

    // 转角平滑代价
    double lambda_yaw_smooth = 300.0;
    for (size_t i = 0; i < N - 1; ++i) {
        J += lambda_yaw_smooth * (P_yaw(i + 1) - P_yaw(i)) * (P_yaw(i + 1) - P_yaw(i));
    }

    // 步态均匀性代价
    double lambda_step_smooth = 30.0;
    for (size_t i = 0; i < N - 1; i++) {
        casadi::MX P_current = P_xy(casadi::Slice(), i);
        casadi::MX P_next = P_xy(casadi::Slice(), i + 1);
        J += lambda_step_smooth * ((P_next(0) - P_current(0)) * (P_next(0) - P_current(0)) + 
                                  (P_next(1) - P_current(1)) * (P_next(1) - P_current(1)));
    }

    // 避免螃蟹步代价
    double lambda_crab_avoid = 80.0;
    for (size_t i = 0; i < N - 2; ++i) {
        casadi::MX delta_x = P_xy(0, i + 2) - P_xy(0, i);
        casadi::MX delta_y = P_xy(1, i + 2) - P_xy(1, i);
        casadi::MX yaw = P_yaw(i);
        casadi::MX lateral_disp = -casadi::MX::sin(yaw) * delta_x + casadi::MX::cos(yaw) * delta_y;
        J += lambda_crab_avoid * (lateral_disp * lateral_disp);
    }

    // 终点方向角跟踪代价
    double lambda_goal_yaw_tracking = 500.0;
    J += lambda_goal_yaw_tracking * (1.0 - casadi::MX::cos(P_yaw(N - 1) - yaw_goal));

    opti.minimize(J);

    // --- 约束定义 ---

    // 初始落脚点约束
    opti.subject_to(P_xy(0, 0) == p_start_support_foot(0));
    opti.subject_to(P_xy(1, 0) == p_start_support_foot(1));
    opti.subject_to(P_yaw(0) == 0); // 假设初始朝向为0

    // 目标点约束
    opti.subject_to(P_xy(0, N - 1) == x_goal);
    opti.subject_to(P_xy(1, N - 1) == y_goal);
    opti.subject_to(P_yaw(N - 1) == yaw_goal);

    // 相邻步转角约束
    double max_step_yaw = M_PI / 12;
    for (size_t i = 0; i < N - 1; ++i) {
        opti.subject_to(casadi::MX::abs(P_yaw(i + 1) - P_yaw(i)) <= max_step_yaw);
    }

    // 双圆运动学约束（正确实现左右脚区分）
    double deta1 = 1.8;   // 内圆圆心偏移量
    double deta2 = 0.35;  // 外圆圆心偏移量
    double dis_th1 = 1.68; // 内圆半径
    double dis_th2 = 0.675; // 外圆半径
    
    // 计算侵蚀后的安全区域边界
    // 侵蚀边距：脚掌长度的一半
    double erosion_margin = 0.1;  // 0.2米脚掌长度的一半
    
    // 侵蚀后的安全区域
    double eroded_stair_start_x = stair_start_x + erosion_margin;
    double eroded_stair_end_x = stair_end_x - erosion_margin;
    double eroded_stair_width = 1.0 - 0.1;  // 原始宽度1米，侵蚀后0.9米
    double eroded_stair_y_min = 0.0 - eroded_stair_width / 2;  // 楼梯中心线y=0
    double eroded_stair_y_max = 0.0 + eroded_stair_width / 2;
    
    for (size_t i = 1; i < N; ++i) {
        // 确定当前支撑脚类型
        bool current_left_support;
        if (i % 2 == 0) {
            current_left_support = !initial_left_support;
        } else {
            current_left_support = initial_left_support;
        }
        
        // 计算支撑脚位置和朝向
        casadi::MX support_center_x = P_xy(0, i - 1);
        casadi::MX support_center_y = P_xy(1, i - 1);
        casadi::MX yaw_support = P_yaw(i - 1);
        casadi::MX cos_yaw = casadi::MX::cos(yaw_support);
        casadi::MX sin_yaw = casadi::MX::sin(yaw_support);
        
        // 计算目标落脚点位置
        casadi::MX target_x = P_xy(0, i);
        casadi::MX target_y = P_xy(1, i);
        
        if (!current_left_support) { // 右脚支撑（左脚摆动）
            // 内圆圆心位置计算
            casadi::MX inner_circle_x = support_center_x - deta1 * sin_yaw;
            casadi::MX inner_circle_y = support_center_y + deta1 * cos_yaw;
            
            // 外圆圆心位置计算
            casadi::MX outer_circle_x = support_center_x - (-deta2) * sin_yaw;
            casadi::MX outer_circle_y = support_center_y + (-deta2) * cos_yaw;
            
            // 计算到内圆圆心的距离
            casadi::MX dx_inner = target_x - inner_circle_x;
            casadi::MX dy_inner = target_y - inner_circle_y;
            casadi::MX dist_inner_sq = dx_inner*dx_inner + dy_inner*dy_inner;
            
            // 计算到外圆圆心的距离
            casadi::MX dx_outer = target_x - outer_circle_x;
            casadi::MX dy_outer = target_y - outer_circle_y;
            casadi::MX dist_outer_sq = dx_outer*dx_outer + dy_outer*dy_outer;
            
            // 双圆约束
            opti.subject_to(dist_inner_sq <= dis_th1 * dis_th1);
            opti.subject_to(dist_outer_sq <= dis_th2 * dis_th2);
            
        } else { // 左脚支撑（右脚摆动）
            // 内圆圆心位置计算（方向相反）
            casadi::MX inner_circle_x = support_center_x - (-deta1) * sin_yaw;
            casadi::MX inner_circle_y = support_center_y + (-deta1) * cos_yaw;
            
            // 外圆圆心位置计算（方向相反）
            casadi::MX outer_circle_x = support_center_x - (deta2) * sin_yaw;
            casadi::MX outer_circle_y = support_center_y + (deta2) * cos_yaw;
            
            // 计算到内圆圆心的距离
            casadi::MX dx_inner = target_x - inner_circle_x;
            casadi::MX dy_inner = target_y - inner_circle_y;
            casadi::MX dist_inner_sq = dx_inner*dx_inner + dy_inner*dy_inner;
            
            // 计算到外圆圆心的距离
            casadi::MX dx_outer = target_x - outer_circle_x;
            casadi::MX dy_outer = target_y - outer_circle_y;
            casadi::MX dist_outer_sq = dx_outer*dx_outer + dy_outer*dy_outer;
            
            // 双圆约束
            opti.subject_to(dist_inner_sq <= dis_th1 * dis_th1);
            opti.subject_to(dist_outer_sq <= dis_th2 * dis_th2);
        }
        
        // 根据您的需求：落脚点必须在侵蚀后的安全区域内
        // 检查落脚点是否在楼梯区域内，如果在则必须满足侵蚀约束
        
        // 判断落脚点是否在楼梯区域内
        casadi::MX in_stair_region = (target_x >= stair_start_x) && (target_x <= stair_end_x) &&
                                    (target_y >= -0.5) && (target_y <= 0.5);
        
        // 如果在楼梯区域内，必须满足侵蚀约束
        double M = 1000.0;  // 大M常数
        casadi::MX safe_x_lower = target_x >= eroded_stair_start_x - M * (1 - in_stair_region);
        casadi::MX safe_x_upper = target_x <= eroded_stair_end_x + M * (1 - in_stair_region);
        casadi::MX safe_y_lower = target_y >= eroded_stair_y_min - M * (1 - in_stair_region);
        casadi::MX safe_y_upper = target_y <= eroded_stair_y_max + M * (1 - in_stair_region);
        
        opti.subject_to(safe_x_lower);
        opti.subject_to(safe_x_upper);
        opti.subject_to(safe_y_lower);
        opti.subject_to(safe_y_upper);
    }

    // ==========================================
    // 6. 初始猜测策略
    // ==========================================
    
    // 区域大按步长，到了楼梯上按楼梯中心点
    for (size_t i = 0; i < N; ++i) {
        double progress = static_cast<double>(i) / (N - 1);
        
        // 线性插值
        double guess_x = p_start_support_foot(0) + progress * (x_goal - p_start_support_foot(0));
        double guess_y = p_start_support_foot(1) + progress * (y_goal - p_start_support_foot(1));
        
        // 检查点是否在楼梯区域内，并确定具体在哪个台阶上
        int step_index = -1;
        if (guess_x >= stair_start_x && guess_x <= stair_end_x && 
            guess_y >= -0.5 && guess_y <= 0.5) {
            // 根据x坐标确定台阶索引
            double x_offset = guess_x - stair_start_x;
            step_index = static_cast<int>(std::floor(x_offset / step_width));
            step_index = std::max(0, std::min(step_count - 1, step_index));
            
            // 如果在台阶上，调整到对应台阶的中心点
            if (step_index >= 0) {
                guess_x = step_centers[step_index].x();  // 台阶中心点的x坐标
                guess_y = step_centers[step_index].y();  // 台阶中心点的y坐标
            }
        }
        
        // 根据支撑脚类型微调y坐标
        bool current_left_support;
        if (i % 2 == 0) {
            current_left_support = !initial_left_support;
        } else {
            current_left_support = initial_left_support;
        }
        
        double foot_offset = 0.1;
        if (current_left_support) {
            guess_y += foot_offset;
        } else {
            guess_y -= foot_offset;
        }
        
        opti.set_initial(P_xy(0, i), guess_x);
        opti.set_initial(P_xy(1, i), guess_y);
        opti.set_initial(P_yaw(i), yaw_goal);
    }

    // ==========================================
    // 7. 求解优化问题
    // ==========================================
    
    // --- 求解 ---
    // 设置IPOPT求解器参数（针对完整双圆模型）
    casadi::Dict ipopt_options;
    ipopt_options["ipopt.max_iter"] = 5000;  // 增加最大迭代次数
    ipopt_options["ipopt.tol"] = 1e-6;       // 收敛容差
    ipopt_options["ipopt.acceptable_tol"] = 1e-4;  // 可接受的容差
    ipopt_options["ipopt.constr_viol_tol"] = 0.001; // 约束违反容差
    ipopt_options["ipopt.linear_solver"] = "mumps"; // 使用更快的线性求解器
    ipopt_options["ipopt.print_level"] = 0;  // 简化输出
    ipopt_options["ipopt.acceptable_iter"] = 10;  // 可接受的迭代次数
    ipopt_options["ipopt.mu_strategy"] = "adaptive"; // 自适应障碍参数策略
    ipopt_options["ipopt.bound_push"] = 0.01; // 边界推动因子
    
    opti.solver("ipopt", ipopt_options);
    
    std::vector<double> res_px, res_py, res_yaw;
    std::vector<Eigen::Vector3d> foot_positions_3d;
    int time_cost = 0;
    
    try {
        casadi::OptiSol sol = opti.solve();
        std::cout << "Optimization Success!" << std::endl;
        
        // 获取优化结果
        res_px = std::vector<double>(sol.value(P_xy(0, casadi::Slice())));
        res_py = std::vector<double>(sol.value(P_xy(1, casadi::Slice())));
        res_yaw = std::vector<double>(sol.value(P_yaw(casadi::Slice())));
        
        // 后处理：根据x,y确定z坐标
        for (size_t i = 0; i < res_px.size(); ++i) {
            double z = 0.0;
            // 检查点是否在楼梯区域内，如果在则计算台阶高度
            if (res_px[i] >= stair_start_x && res_px[i] <= stair_end_x && 
                res_py[i] >= -0.5 && res_py[i] <= 0.5) {
                // 根据x坐标确定台阶索引
                double x_offset = res_px[i] - stair_start_x;
                int step_index = static_cast<int>(std::floor(x_offset / step_width));
                step_index = std::max(0, std::min(step_count - 1, step_index));
                z = (step_index + 1) * step_height;
            }
            foot_positions_3d.push_back(Eigen::Vector3d(res_px[i], res_py[i], z));
            
            std::cout << "Step " << i << ": (" << res_px[i] << ", " 
                      << res_py[i] << ", " << z << "), yaw: " << res_yaw[i] << std::endl;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Optimization Time: " << duration << " ms" << std::endl;
        time_cost = duration;

    } catch(std::exception& e) {
        std::cerr << "Optimization Failed: " << e.what() << std::endl;
        return -1;
    }

    // ==========================================
    // 8. 可视化
    // ==========================================
    
    // 准备落脚点数据（2D用于俯视图）
    std::vector<Eigen::Vector2d> foot_positions_2d;
    for (const auto& pos : foot_positions_3d) {
        foot_positions_2d.push_back(Eigen::Vector2d(pos(0), pos(1)));
    }
    
    // 准备可达区域数据（双圆模型）
    std::vector<Eigen::Vector2d> reachable_regions;
    double inner_radius = 1.68;   // 内圆半径
    double outer_radius = 0.675;  // 外圆半径
    
    // 简化实现：直接使用落脚点作为可达区域中心
    for (size_t i = 1; i < foot_positions_2d.size(); ++i) {
        // 添加落脚点作为可达区域点
        reachable_regions.push_back(foot_positions_2d[i-1]);
        reachable_regions.push_back(foot_positions_2d[i]);
    }
    
    // 生成时间戳用于文件名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::string timestamp = std::to_string(time_t);
    
    // 俯视图
    plt::figure_size(1200, 800);
    
    // 画楼梯原始区域（只用线框，不填充）
    std::vector<double> stair_x, stair_y;
    for (int i = 0; i < step_count; ++i) {
        // 画每个台阶的边界
        const auto& vertices = step_planes[i];
        for (size_t j = 0; j < vertices.size(); ++j) {
            stair_x.push_back(vertices[j].x());
            stair_y.push_back(vertices[j].y());
        }
        // 闭合多边形
        stair_x.push_back(vertices[0].x());
        stair_y.push_back(vertices[0].y());
        stair_x.push_back(NAN);  // 分隔符
        stair_y.push_back(NAN);
    }
    plt::plot(stair_x, stair_y, "k-");
    
    // 画侵蚀后的区域（只用线框，不填充）
    std::vector<double> eroded_x, eroded_y;
    for (int i = 0; i < step_count; ++i) {
        // 画每个台阶的侵蚀边界
        const auto& vertices = eroded_step_planes[i];
        for (size_t j = 0; j < vertices.size(); ++j) {
            eroded_x.push_back(vertices[j].x());
            eroded_y.push_back(vertices[j].y());
        }
        // 闭合多边形
        eroded_x.push_back(vertices[0].x());
        eroded_y.push_back(vertices[0].y());
        eroded_x.push_back(NAN);  // 分隔符
        eroded_y.push_back(NAN);
    }
    if (!eroded_x.empty()) {
        plt::plot(eroded_x, eroded_y, "r--");
    }
    
    // 画落脚点轨迹
    std::vector<double> traj_x, traj_y;
    for (const auto& pos : foot_positions_2d) {
        traj_x.push_back(pos(0));
        traj_y.push_back(pos(1));
    }
    plt::plot(traj_x, traj_y, "b-o");
    
    // 画落脚点箭头（表示朝向）
    std::vector<double> arrow_x, arrow_y, arrow_u, arrow_v;
    double arrow_len = 0.1;
    for (size_t i = 0; i < foot_positions_2d.size(); ++i) {
        arrow_x.push_back(foot_positions_2d[i](0));
        arrow_y.push_back(foot_positions_2d[i](1));
        arrow_u.push_back(arrow_len * cos(res_yaw[i]));
        arrow_v.push_back(arrow_len * sin(res_yaw[i]));
    }
    plt::quiver(arrow_x, arrow_y, arrow_u, arrow_v);
    
    // 添加图例和标签
    plt::legend();
    plt::title("Stair Footstep Planning - Top View");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::grid(true);
    
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::title("Stair Footstep Planning - Top View (" + std::to_string(N) + " steps)");
    plt::grid(true);
    plt::axis("equal");
    
    // 保存俯视图
    std::string topview_filename = "stair_topview_" + timestamp + ".png";
    plt::save(topview_filename);
    std::cout << "Top view saved as: " << topview_filename << std::endl;
    
    // 剖面图
    plt::figure_size(1200, 600);
    
    std::vector<double> profile_x, profile_z;
    for (const auto& pos : foot_positions_3d) {
        profile_x.push_back(pos(0));
        profile_z.push_back(pos(2));
    }
    
    // 画楼梯完整剖面（包括台阶）
    // 生成楼梯中心线
    std::vector<Eigen::Vector3d> center_line;
    for (int i = 0; i < step_count; ++i) {
        double x_start = 0.8 + i * step_width;
        double x_end = x_start + step_width;
        double z = (i + 1) * step_height;
        
        // 台阶起点
        center_line.push_back(Eigen::Vector3d(x_start, 0.0, z));
        // 台阶终点
        center_line.push_back(Eigen::Vector3d(x_end, 0.0, z));
    }
    
    // 绘制楼梯的阶梯状结构
    for (size_t i = 0; i < center_line.size() - 1; ++i) {
        // 水平部分（台阶面）
        std::vector<double> level_x = {center_line[i](0), center_line[i+1](0)};
        std::vector<double> level_z = {center_line[i](2), center_line[i](2)};
        plt::plot(level_x, level_z, "k-");
        
        // 垂直部分（台阶边缘）
        if (i < center_line.size() - 1) {
            std::vector<double> vertical_x = {center_line[i+1](0), center_line[i+1](0)};
            std::vector<double> vertical_z = {center_line[i](2), center_line[i+1](2)};
            plt::plot(vertical_x, vertical_z, "k-");
        }
    }
    
    // 添加最后一个台阶的水平面
    if (center_line.size() > 1) {
        std::vector<double> last_level_x = {center_line.back()(0), center_line.back()(0) + step_width};
        std::vector<double> last_level_z = {center_line.back()(2), center_line.back()(2)};
        plt::plot(last_level_x, last_level_z, "k-");
    }
    
    // 画每个台阶的侵蚀范围（剖面）
    // 计算每个台阶的侵蚀范围
    double foot_length = 0.2;  // 脚掌长度
    double foot_width = 0.1;  // 脚掌宽度
    
    // 侵蚀范围：在台阶长度方向（x方向）上减少脚掌长度的一半
    double erosion_margin_x = foot_length / 2.0;
    
    for (size_t i = 0; i < center_line.size() - 1; ++i) {
        double x_start = center_line[i](0);
        double x_end = center_line[i+1](0);
        double z_level = center_line[i](2);
        
        // 侵蚀后的台阶范围（x方向上的安全区域）
        double eroded_x_start = x_start + erosion_margin_x;
        double eroded_x_end = x_end - erosion_margin_x;
        
        // 确保侵蚀后的范围有效
        if (eroded_x_start < eroded_x_end) {
            // 侵蚀后的台阶上边界（安全区域的上边界）
            std::vector<double> eroded_top_x = {eroded_x_start, eroded_x_end};
            std::vector<double> eroded_top_z = {z_level, z_level};
            plt::plot(eroded_top_x, eroded_top_z, "g--");
            
            // 侵蚀后的台阶下边界（安全区域的下边界）
            std::vector<double> eroded_bottom_x = {eroded_x_start, eroded_x_end};
            std::vector<double> eroded_bottom_z = {z_level - step_height, z_level - step_height};
            plt::plot(eroded_bottom_x, eroded_bottom_z, "g--");
            
            // 连接侵蚀区域的垂直边界
            std::vector<double> eroded_left_x = {eroded_x_start, eroded_x_start};
            std::vector<double> eroded_left_z = {z_level - step_height, z_level};
            plt::plot(eroded_left_x, eroded_left_z, "g--");
            
            std::vector<double> eroded_right_x = {eroded_x_end, eroded_x_end};
            std::vector<double> eroded_right_z = {z_level - step_height, z_level};
            plt::plot(eroded_right_x, eroded_right_z, "g--");
        }
    }
    
    // 画落脚点剖面
    plt::plot(profile_x, profile_z, "r-o");
    
    // 标记每个落脚点的高度
    for (size_t i = 0; i < profile_x.size(); ++i) {
        plt::annotate("Step " + std::to_string(i), profile_x[i], profile_z[i]);
    }
    
    plt::xlabel("X (m)");
    plt::ylabel("Z (m)");
    plt::title("Stair Footstep Planning - Profile View");
    plt::legend();
    plt::grid(true);
    
    // 保存剖面图
    std::string profile_filename = "stair_profile_" + timestamp + ".png";
    plt::save(profile_filename);
    std::cout << "Profile view saved as: " << profile_filename << std::endl;
    
    // 显示所有图形
    plt::show();
    
    LOG(INFO) << "Stair footstep planning completed successfully!";
    
    return 0;
}