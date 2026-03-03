#ifndef STAIR_GEOMETRY_H
#define STAIR_GEOMETRY_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <casadi/casadi.hpp>

class StairGeometry {
private:
    Eigen::Vector3d center_point;
    std::vector<Eigen::Vector3d> polygon_vertices;
    double step_height, step_width;
    int step_count;
    
    // 脚掌几何参数
    double foot_length, foot_width;
    
public:
    // 构造函数：从中心点和多边形顶点初始化
    StairGeometry(const Eigen::Vector3d& center, 
                  const std::vector<Eigen::Vector3d>& vertices,
                  double height, double width, int count,
                  double foot_len = 0.2, double foot_wid = 0.1);
    
    // 侵蚀计算：将脚掌几何投影到楼梯表面
    std::vector<Eigen::Vector3d> erodeWithFootprint(const Eigen::Vector3d& foot_center, 
                                                   double yaw);
    
    // 可达区域计算：结合双圆模型和楼梯侵蚀
    std::vector<Eigen::Vector3d> calculateReachableRegion(const Eigen::Vector3d& current_foot,
                                                          double yaw,
                                                          double inner_radius,
                                                          double outer_radius);
    
    // 高度查询：根据x,y确定z坐标
    double getHeightAtPosition(double x, double y) const;
    
    // 检查点是否在楼梯区域内
    bool isPointInStairRegion(double x, double y) const;
    
    // 获取楼梯台阶信息
    int getStepIndexAtPosition(double x, double y) const;
    
    // 获取楼梯中心线
    std::vector<Eigen::Vector3d> getStairCenterLine() const;
    
    // 可视化数据生成
    std::vector<std::vector<double>> getVisualizationData() const;
    
    // 获取侵蚀后的可达区域边界
    std::vector<Eigen::Vector3d> getErodedBoundary(const Eigen::Vector3d& foot_center, 
                                                  double yaw);
    
    // CasADi MX版本的可达性检查
    casadi::MX isReachableMX(const casadi::MX& current_pos, 
                           const casadi::MX& current_yaw,
                           const casadi::MX& target_pos,
                           double inner_radius,
                           double outer_radius) const;
    
    // 获取属性
    Eigen::Vector3d getCenter() const { return center_point; }
    double getStepHeight() const { return step_height; }
    double getStepWidth() const { return step_width; }
    int getStepCount() const { return step_count; }
    
private:
    // 内部辅助函数
    bool pointInPolygon(double x, double y, const std::vector<Eigen::Vector3d>& polygon) const;
    double pointToLineDistance(double x, double y, 
                             double x1, double y1, double x2, double y2) const;
    std::vector<Eigen::Vector3d> minkowskiDifference(const std::vector<Eigen::Vector3d>& poly1,
                                                     const std::vector<Eigen::Vector3d>& poly2) const;
};

#endif // STAIR_GEOMETRY_H