#include <open3d/Open3D.h>
#include <iostream>
#include <cmath>
#include <unordered_map>

using namespace open3d;

bool SaveVoxelGridAsPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& voxelized_cloud, const std::string& filename) {
    return open3d::io::WritePointCloud(filename, *voxelized_cloud);
}

// Structure to store cylindrical voxel indices
struct CylindricalVoxel {
    int radial_index;
    int angular_index;
    int height_index;

    // Custom hash function for using this struct in an unordered_map
    bool operator==(const CylindricalVoxel& other) const {
        return radial_index == other.radial_index &&
               angular_index == other.angular_index &&
               height_index == other.height_index;
    }
};

struct CylindricalVoxelHash {
    std::size_t operator()(const CylindricalVoxel& voxel) const {
        return std::hash<int>()(voxel.radial_index) ^ std::hash<int>()(voxel.angular_index << 1) ^ std::hash<int>()(voxel.height_index << 2);
    }
};

// Helper structure to accumulate color and count for averaging
struct ColorAccumulator {
    Eigen::Vector3d color_sum = Eigen::Vector3d(0, 0, 0);
    int count = 0;

    void AddColor(const Eigen::Vector3d& color) {
        color_sum += color;
        ++count;
    }

    Eigen::Vector3d GetAverageColor() const {
        return count > 0 ? color_sum / count : Eigen::Vector3d(0, 0, 0);
    }
};

// Convert Cartesian coordinates to cylindrical coordinates
void CartesianToCylindrical(const Eigen::Vector3d& point, double& r, double& theta, double& z) {
    r = std::sqrt(point(0) * point(0) + point(1) * point(1));
    theta = std::atan2(point(1), point(0));
    if (theta < 0) theta += 2 * M_PI; // Normalize angle to [0, 2*pi]
    z = point(2);
}

std::shared_ptr<geometry::PointCloud> CylindricalVoxelizeMeshWithColor(
    const std::shared_ptr<geometry::TriangleMesh>& mesh,
    double radial_resolution, double angular_resolution, double height_resolution) {

    // Hash map to store unique cylindrical voxels and their accumulated colors
    std::unordered_map<CylindricalVoxel, ColorAccumulator, CylindricalVoxelHash> cylindrical_voxel_map;

    // Iterate through each vertex in the mesh
    for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
        Eigen::Vector3d vertex = mesh->vertices_[i];
        Eigen::Vector3d color = mesh->vertex_colors_[i];

        // Convert vertex position to cylindrical coordinates
        double r, theta, z;
        CartesianToCylindrical(vertex, r, theta, z);

        // Compute the cylindrical voxel indices
        int radial_index = static_cast<int>(r / radial_resolution);
        int angular_index = static_cast<int>(theta / angular_resolution);
        int height_index = static_cast<int>(z / height_resolution);

        // Define the cylindrical voxel
        CylindricalVoxel voxel{radial_index, angular_index, height_index};

        // Accumulate color for this voxel
        cylindrical_voxel_map[voxel].AddColor(color);
    }

    // Convert the cylindrical voxel map to a PointCloud for visualization
    auto voxelized_cloud = std::make_shared<geometry::PointCloud>();

    for (const auto& item : cylindrical_voxel_map) {
        CylindricalVoxel voxel = item.first;
        ColorAccumulator color_accumulator = item.second;

        // Get the average color for this voxel
        Eigen::Vector3d average_color = color_accumulator.GetAverageColor();

        // Convert the cylindrical voxel center back to Cartesian coordinates for visualization
        double r = (voxel.radial_index + 0.5) * radial_resolution;
        double theta = (voxel.angular_index + 0.5) * angular_resolution;
        double z = (voxel.height_index + 0.5) * height_resolution;

        // Cartesian coordinates
        Eigen::Vector3d cartesian_point(r * cos(theta), r * sin(theta), z);

        // Add the voxel center point to the point cloud
        voxelized_cloud->points_.push_back(cartesian_point);
        voxelized_cloud->colors_.push_back(average_color);
    }

    return voxelized_cloud;
}

int main(int argc, char** argv) {

    // Load the mesh from a .obj file
    auto mesh = std::make_shared<geometry::TriangleMesh>();
    if (!io::ReadTriangleMesh("/home/boolmon/Documents/Models/kurumi/kurumi.ply", *mesh)) {
        std::cerr << "Failed to load mesh!" << std::endl;
        return -1;
    }

    // Ensure the mesh has vertex colors loaded
    if (mesh->vertex_colors_.empty()) {
        std::cerr << "Mesh does not have colors!" << std::endl;
        return -1;
    }

    // Set cylindrical grid resolutions
    double radial_resolution = 0.1;   // radial step
    double angular_resolution = M_PI / 72.0; // 10 degrees
    double height_resolution = 0.1;   // height step

    // Voxelize the mesh with cylindrical grid and colors
    auto voxelized_cloud = CylindricalVoxelizeMeshWithColor(mesh, radial_resolution, angular_resolution, height_resolution);

    // Save the voxel grid as a .ply file
    if (SaveVoxelGridAsPointCloud(voxelized_cloud, "voxel_model_cylinder.ply")) {
        std::cout << "Successfully saved voxel grid as a point cloud." << std::endl;
    } else {
        std::cerr << "Failed to save voxel grid as a point cloud." << std::endl;
    }

    // Visualize the result
    visualization::DrawGeometries({voxelized_cloud}, "Cylindrical Voxelized Model with Color");

    return 0;
}