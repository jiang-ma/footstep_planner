#ifndef VISUALIZATION_3D_H
#define VISUALIZATION_3D_H

#include "stair_geometry.h"
#include <Eigen/Core>
#include <vector>
#include <string>

class Visualization3D {
public:
    // 俯视图可视化
    static void plotTopView(const std::vector<Eigen::Vector2d>& foot_positions,
                           const StairGeometry& stairs,
                           const std::vector<Eigen::Vector2d>& reachable_regions,
                           const std::string& filename = "");
    
    // 剖面图可视化
    static void plotProfileView(const std::vector<Eigen::Vector3d>& foot_positions,
                               const StairGeometry& stairs,
                               const std::string& filename = "");
    
    // 3D整体可视化
    static void plot3DView(const std::vector<Eigen::Vector3d>& foot_positions,
                          const StairGeometry& stairs,
                          const std::string& filename = "");
    
    // 保存图片
    static void savePlot(const std::string& filename);
    
private:
    // 内部辅助函数
    static std::vector<std::vector<double>> prepareStairData(const StairGeometry& stairs);
    static std::vector<std::vector<double>> prepareFootprintData(const std::vector<Eigen::Vector3d>& positions);
    static std::vector<std::vector<double>> prepareReachableRegionData(const std::vector<Eigen::Vector2d>& regions);
};

#endif // VISUALIZATION_3D_H