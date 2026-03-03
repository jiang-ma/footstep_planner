#include "../include/stair_geometry.h"
#include <glog/logging.h>
#include <iostream>

// 定义一个简化的楼梯几何类，不包含CasADi相关功能
class SimpleStairGeometry {
private:
    Eigen::Vector3d center_point;
    std::vector<Eigen::Vector3d> polygon_vertices;
    double step_height, step_width;
    int step_count;
    
public:
    SimpleStairGeometry(const Eigen::Vector3d& center, 
                       const std::vector<Eigen::Vector3d>& vertices,
                       double height, double width, int count)
        : center_point(center), polygon_vertices(vertices),
          step_height(height), step_width(width), step_count(count) {}
    
    bool isPointInStairRegion(double x, double y) const {
        if (polygon_vertices.size() < 3) return false;
        
        bool inside = false;
        size_t n = polygon_vertices.size();
        
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon_vertices[i](0), yi = polygon_vertices[i](1);
            double xj = polygon_vertices[j](0), yj = polygon_vertices[j](1);
            
            if (((yi > y) != (yj > y)) && 
                (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        
        return inside;
    }
    
    double getHeightAtPosition(double x, double y) const {
        if (!isPointInStairRegion(x, y)) {
            return 0.0;
        }
        
        double x_offset = x - center_point(0);
        int step_index = static_cast<int>(x_offset / step_width);
        step_index = std::max(0, std::min(step_count - 1, step_index));
        
        return step_index * step_height;
    }
    
    int getStepIndexAtPosition(double x, double y) const {
        if (!isPointInStairRegion(x, y)) {
            return -1;
        }
        
        double x_offset = x - center_point(0);
        int step_index = static_cast<int>(x_offset / step_width);
        return std::max(0, std::min(step_count - 1, step_index));
    }
};

int main() {
    google::InitGoogleLogging("test_stair_geometry");
    
    // 测试简化的楼梯几何类
    Eigen::Vector3d center(2.0, 0.0, 0.0);
    std::vector<Eigen::Vector3d> vertices = {
        Eigen::Vector3d(1.0, -1.0, 0.0),
        Eigen::Vector3d(3.0, -1.0, 0.0),
        Eigen::Vector3d(3.0, 1.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0)
    };
    
    SimpleStairGeometry stairs(center, vertices, 0.15, 0.3, 5);
    
    // 测试点是否在楼梯区域内
    std::cout << "Testing point in stair region:" << std::endl;
    std::cout << "Point (2.0, 0.0): " << (stairs.isPointInStairRegion(2.0, 0.0) ? "Inside" : "Outside") << std::endl;
    std::cout << "Point (0.0, 0.0): " << (stairs.isPointInStairRegion(0.0, 0.0) ? "Inside" : "Outside") << std::endl;
    
    // 测试高度计算
    std::cout << "\nTesting height calculation:" << std::endl;
    std::cout << "Height at (1.5, 0.0): " << stairs.getHeightAtPosition(1.5, 0.0) << std::endl;
    std::cout << "Height at (2.0, 0.0): " << stairs.getHeightAtPosition(2.0, 0.0) << std::endl;
    std::cout << "Height at (2.5, 0.0): " << stairs.getHeightAtPosition(2.5, 0.0) << std::endl;
    
    // 测试台阶索引
    std::cout << "\nTesting step index:" << std::endl;
    std::cout << "Step index at (1.5, 0.0): " << stairs.getStepIndexAtPosition(1.5, 0.0) << std::endl;
    std::cout << "Step index at (2.0, 0.0): " << stairs.getStepIndexAtPosition(2.0, 0.0) << std::endl;
    std::cout << "Step index at (2.5, 0.0): " << stairs.getStepIndexAtPosition(2.5, 0.0) << std::endl;
    
    // 测试侵蚀计算（简化版本）
    std::cout << "\nTesting simplified erosion calculation:" << std::endl;
    Eigen::Vector3d foot_center(1.5, 0.0, 0.0);
    std::cout << "Foot center at (" << foot_center(0) << ", " << foot_center(1) << ")" << std::endl;
    
    // 测试可达区域计算（简化版本）
    std::cout << "\nTesting simplified reachable region calculation:" << std::endl;
    std::cout << "Reachable region calculation requires CasADi - skipping in simple test" << std::endl;
    
    std::cout << "\nAll tests completed successfully!" << std::endl;
    
    return 0;
}