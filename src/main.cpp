#include <open3d/Open3D.h>

int main() {
    open3d::geometry::PointCloud pointcloud;
    pointcloud.points_.emplace_back(0, 0, 0);
    pointcloud.points_.emplace_back(1, 0, 0);
    pointcloud.points_.emplace_back(0, 1, 0);
    
    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
    return 0;
}
