#include "../include/stair_geometry.h"
#include <cmath>
#include <algorithm>
#include <iostream>

StairGeometry::StairGeometry(const Eigen::Vector3d& center, 
                           const std::vector<Eigen::Vector3d>& vertices,
                           double height, double width, int count,
                           double foot_len, double foot_wid)
    : center_point(center), polygon_vertices(vertices),
      step_height(height), step_width(width), step_count(count),
      foot_length(foot_len), foot_width(foot_wid) {
    
    // 验证输入参数
    if (polygon_vertices.size() < 3) {
        std::cerr << "Warning: Stair polygon should have at least 3 vertices" << std::endl;
    }
    
    if (step_height <= 0 || step_width <= 0 || step_count <= 0) {
        std::cerr << "Warning: Invalid stair parameters" << std::endl;
    }
}

std::vector<Eigen::Vector3d> StairGeometry::erodeWithFootprint(const Eigen::Vector3d& foot_center, 
                                                              double yaw) {
    std::vector<Eigen::Vector3d> eroded_vertices;
    
    // 1. 创建脚掌几何（矩形）
    std::vector<Eigen::Vector3d> foot_polygon;
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    
    // 脚掌矩形的四个角点（相对于脚掌中心）
    Eigen::Vector3d corners[4] = {
        Eigen::Vector3d(foot_length/2, foot_width/2, 0),
        Eigen::Vector3d(foot_length/2, -foot_width/2, 0),
        Eigen::Vector3d(-foot_length/2, -foot_width/2, 0),
        Eigen::Vector3d(-foot_length/2, foot_width/2, 0)
    };
    
    // 旋转和平移脚掌几何
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d rotated_corner(
            corners[i](0) * cos_yaw - corners[i](1) * sin_yaw,
            corners[i](0) * sin_yaw + corners[i](1) * cos_yaw,
            0
        );
        foot_polygon.push_back(foot_center + rotated_corner);
    }
    
    // 2. 计算Minkowski差（侵蚀）
    eroded_vertices = minkowskiDifference(polygon_vertices, foot_polygon);
    
    return eroded_vertices;
}

std::vector<Eigen::Vector3d> StairGeometry::calculateReachableRegion(const Eigen::Vector3d& current_foot,
                                                                    double yaw,
                                                                    double inner_radius,
                                                                    double outer_radius) {
    std::vector<Eigen::Vector3d> reachable_region;
    
    // 1. 计算侵蚀后的楼梯区域
    std::vector<Eigen::Vector3d> eroded_stairs = erodeWithFootprint(current_foot, yaw);
    
    // 2. 基于双圆模型计算可达区域
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    
    // 内圆和外圆的圆心偏移（根据支撑脚类型）
    double inner_offset_x, inner_offset_y, outer_offset_x, outer_offset_y;
    
    // 这里简化处理，实际应根据支撑脚类型调整
    inner_offset_x = -1.8 * sin_yaw;
    inner_offset_y = 1.8 * cos_yaw;
    outer_offset_x = 0.35 * sin_yaw;
    outer_offset_y = -0.35 * cos_yaw;
    
    Eigen::Vector3d inner_center = current_foot + Eigen::Vector3d(inner_offset_x, inner_offset_y, 0);
    Eigen::Vector3d outer_center = current_foot + Eigen::Vector3d(outer_offset_x, outer_offset_y, 0);
    
    // 3. 计算双圆区域与侵蚀楼梯区域的交集
    // 这里简化实现：采样检查点是否同时在两个区域内
    const int sample_count = 100;
    for (int i = 0; i < sample_count; ++i) {
        double angle = 2 * M_PI * i / sample_count;
        
        // 检查内圆边界点
        double x_inner = inner_center(0) + inner_radius * cos(angle);
        double y_inner = inner_center(1) + inner_radius * sin(angle);
        
        // 检查外圆边界点
        double x_outer = outer_center(0) + outer_radius * cos(angle);
        double y_outer = outer_center(1) + outer_radius * sin(angle);
        
        // 如果点在侵蚀楼梯区域内，则加入可达区域
        if (pointInPolygon(x_inner, y_inner, eroded_stairs)) {
            reachable_region.push_back(Eigen::Vector3d(x_inner, y_inner, getHeightAtPosition(x_inner, y_inner)));
        }
        
        if (pointInPolygon(x_outer, y_outer, eroded_stairs)) {
            reachable_region.push_back(Eigen::Vector3d(x_outer, y_outer, getHeightAtPosition(x_outer, y_outer)));
        }
    }
    
    return reachable_region;
}

double StairGeometry::getHeightAtPosition(double x, double y) const {
    // 简化实现：根据x坐标确定台阶高度
    // 实际应根据楼梯几何精确计算
    
    if (!isPointInStairRegion(x, y)) {
        return 0.0; // 不在楼梯区域内，高度为0
    }
    
    // 计算楼梯实际起始位置
    double total_stair_length = step_count * step_width;
    double stair_start_x = center_point(0) - total_stair_length / 2;
    
    // 计算相对于楼梯起始点的x偏移
    double x_offset = x - stair_start_x;
    
    // 根据x偏移确定台阶索引
    // 使用floor函数确保正确处理边界情况
    int step_index = static_cast<int>(std::floor(x_offset / step_width));
    
    // 确保台阶索引在有效范围内
    // 对于step_count=8，台阶索引应该是0-7（对应高度0.15-1.2米）
    step_index = std::max(0, std::min(step_count - 1, step_index));
    
    // 计算实际台阶高度（从第一个台阶开始）
    return (step_index + 1) * step_height;
}

bool StairGeometry::isPointInStairRegion(double x, double y) const {
    return pointInPolygon(x, y, polygon_vertices);
}

int StairGeometry::getStepIndexAtPosition(double x, double y) const {
    if (!isPointInStairRegion(x, y)) {
        return -1; // 不在楼梯区域内
    }
    
    // 计算楼梯实际起始位置
    double total_stair_length = step_count * step_width;
    double stair_start_x = center_point(0) - total_stair_length / 2;
    
    // 计算相对于楼梯起始点的x偏移
    double x_offset = x - stair_start_x;
    int step_index = static_cast<int>(x_offset / step_width);
    return std::max(0, std::min(step_count - 1, step_index));
}

std::vector<Eigen::Vector3d> StairGeometry::getStairCenterLine() const {
    std::vector<Eigen::Vector3d> center_line;
    
    // 计算楼梯实际起始位置
    double total_stair_length = step_count * step_width;
    double stair_start_x = center_point(0) - total_stair_length / 2;
    
    for (int i = 0; i < step_count; ++i) {
        double x = stair_start_x + i * step_width;
        double y = center_point(1);
        double z = (i + 1) * step_height;  // 从第一个台阶开始
        center_line.push_back(Eigen::Vector3d(x, y, z));
    }
    
    // 添加最后一个台阶的终点
    double x_end = stair_start_x + step_count * step_width;
    double z_end = step_count * step_height;  // 最后一个台阶的高度
    center_line.push_back(Eigen::Vector3d(x_end, center_point(1), z_end));
    
    return center_line;
}

std::vector<std::vector<double>> StairGeometry::getVisualizationData() const {
    std::vector<std::vector<double>> data;
    
    // 提取多边形顶点坐标
    std::vector<double> poly_x, poly_y, poly_z;
    for (const auto& vertex : polygon_vertices) {
        poly_x.push_back(vertex(0));
        poly_y.push_back(vertex(1));
        poly_z.push_back(vertex(2));
    }
    
    // 添加闭合点
    if (!polygon_vertices.empty()) {
        poly_x.push_back(polygon_vertices[0](0));
        poly_y.push_back(polygon_vertices[0](1));
        poly_z.push_back(polygon_vertices[0](2));
    }
    
    data.push_back(poly_x);
    data.push_back(poly_y);
    data.push_back(poly_z);
    
    return data;
}

std::vector<Eigen::Vector3d> StairGeometry::getErodedBoundary(const Eigen::Vector3d& foot_center, 
                                                             double yaw) {
    return erodeWithFootprint(foot_center, yaw);
}

casadi::MX StairGeometry::isReachableMX(const casadi::MX& current_pos, 
                                      const casadi::MX& current_yaw,
                                      const casadi::MX& target_pos,
                                      double inner_radius,
                                      double outer_radius) const {
    // 简化实现：检查目标点是否在双圆区域内
    // 实际应结合楼梯侵蚀计算
    
    casadi::MX dx = target_pos(0) - current_pos(0);
    casadi::MX dy = target_pos(1) - current_pos(1);
    casadi::MX dist_sq = dx*dx + dy*dy;
    
    // 检查是否在内外圆半径之间
    casadi::MX in_outer_circle = dist_sq <= outer_radius * outer_radius;
    casadi::MX in_inner_circle = dist_sq >= inner_radius * inner_radius;
    
    return in_outer_circle && in_inner_circle;
}

// 内部辅助函数实现
bool StairGeometry::pointInPolygon(double x, double y, const std::vector<Eigen::Vector3d>& polygon) const {
    if (polygon.size() < 3) return false;
    
    bool inside = false;
    size_t n = polygon.size();
    
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i](0), yi = polygon[i](1);
        double xj = polygon[j](0), yj = polygon[j](1);
        
        if (((yi > y) != (yj > y)) && 
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    
    return inside;
}

double StairGeometry::pointToLineDistance(double x, double y, 
                                        double x1, double y1, double x2, double y2) const {
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = (len_sq != 0) ? dot / len_sq : -1;
    
    double xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    
    double dx = x - xx;
    double dy = y - yy;
    return sqrt(dx * dx + dy * dy);
}

std::vector<Eigen::Vector3d> StairGeometry::minkowskiDifference(const std::vector<Eigen::Vector3d>& poly1,
                                                               const std::vector<Eigen::Vector3d>& poly2) const {
    std::vector<Eigen::Vector3d> result;
    
    // 简化实现：计算凸包近似
    // 实际应使用更精确的Minkowski差算法
    
    if (poly1.empty() || poly2.empty()) {
        return result;
    }
    
    // 找到poly1的边界
    double min_x = poly1[0](0), max_x = poly1[0](0);
    double min_y = poly1[0](1), max_y = poly1[0](1);
    
    for (const auto& p : poly1) {
        min_x = std::min(min_x, p(0));
        max_x = std::max(max_x, p(0));
        min_y = std::min(min_y, p(1));
        max_y = std::max(max_y, p(1));
    }
    
    // 找到poly2的边界
    for (const auto& p : poly2) {
        min_x = std::min(min_x, p(0) - foot_length/2);
        max_x = std::max(max_x, p(0) + foot_length/2);
        min_y = std::min(min_y, p(1) - foot_width/2);
        max_y = std::max(max_y, p(1) + foot_width/2);
    }
    
    // 创建简化的侵蚀边界（矩形）
    double safety_margin = 0.05; // 安全边界
    result.push_back(Eigen::Vector3d(min_x + safety_margin, min_y + safety_margin, 0));
    result.push_back(Eigen::Vector3d(max_x - safety_margin, min_y + safety_margin, 0));
    result.push_back(Eigen::Vector3d(max_x - safety_margin, max_y - safety_margin, 0));
    result.push_back(Eigen::Vector3d(min_x + safety_margin, max_y - safety_margin, 0));
    
    return result;
}