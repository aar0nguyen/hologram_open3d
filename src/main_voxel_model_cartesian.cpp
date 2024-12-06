#include <open3d/Open3D.h>
#include <iostream>

using namespace open3d;


// Helper function to check if a point is inside a triangle in 3D
bool IsPointInTriangle(const Eigen::Vector3d& p,
                       const Eigen::Vector3d& v0,
                       const Eigen::Vector3d& v1,
                       const Eigen::Vector3d& v2) {
    // Barycentric coordinates for point-in-triangle test
    Eigen::Vector3d u = v1 - v0;
    Eigen::Vector3d v = v2 - v0;
    Eigen::Vector3d w = p - v0;

    double uu = u.dot(u);
    double uv = u.dot(v);
    double vv = v.dot(v);
    double wu = w.dot(u);
    double wv = w.dot(v);
    double denom = uv * uv - uu * vv;

    double s = (uv * wv - vv * wu) / denom;
    double t = (uv * wu - uu * wv) / denom;

    // If the point lies within the triangle, s and t should both be >= 0 and s + t <= 1
    return (s >= 0) && (t >= 0) && (s + t <= 1);
}

std::shared_ptr<geometry::VoxelGrid> VoxelizeMeshWithColor(
    const std::shared_ptr<geometry::TriangleMesh>& mesh,
    double voxel_size) {
    // Initialize an empty VoxelGrid
    auto voxel_grid = std::make_shared<geometry::VoxelGrid>();
    voxel_grid->voxel_size_ = voxel_size;

    // Set up the voxel grid bounds based on the mesh
    Eigen::Vector3d min_bound = mesh->GetMinBound();
    Eigen::Vector3d max_bound = mesh->GetMaxBound();
    voxel_grid->origin_ = min_bound;

    // Calculate the voxel grid dimensions
    Eigen::Vector3i voxel_dimensions = ((max_bound - min_bound) / voxel_size).cast<int>();

    // Iterate through each triangle and each voxel, transferring color
    for (size_t i = 0; i < mesh->triangles_.size(); ++i) {
        // Get the vertices of the triangle
        Eigen::Vector3i triangle = mesh->triangles_[i];
        Eigen::Vector3d v0 = mesh->vertices_[triangle(0)];
        Eigen::Vector3d v1 = mesh->vertices_[triangle(1)];
        Eigen::Vector3d v2 = mesh->vertices_[triangle(2)];

        // Compute the triangle's center color (you can choose other strategies)
        Eigen::Vector3d color = (mesh->vertex_colors_[triangle(0)] +
                                 mesh->vertex_colors_[triangle(1)] +
                                 mesh->vertex_colors_[triangle(2)]) / 3.0;

        // Find the bounding box of the triangle in voxel space
        Eigen::Vector3d min_corner = v0.cwiseMin(v1).cwiseMin(v2);
        Eigen::Vector3d max_corner = v0.cwiseMax(v1).cwiseMax(v2);

        // Convert bounds to voxel indices
        Eigen::Vector3i min_voxel = ((min_corner - min_bound) / voxel_size).cast<int>();
        Eigen::Vector3i max_voxel = ((max_corner - min_bound) / voxel_size).cast<int>();

        // Traverse through the voxels in the triangle's bounding box
        for (int x = min_voxel(0); x <= max_voxel(0); ++x) {
            for (int y = min_voxel(1); y <= max_voxel(1); ++y) {
                for (int z = min_voxel(2); z <= max_voxel(2); ++z) {
                    // Get voxel center in world space
                    Eigen::Vector3d voxel_center = min_bound + Eigen::Vector3d(x, y, z) * voxel_size;

                    // Check if voxel center is inside the triangle
                    if (IsPointInTriangle(voxel_center, v0, v1, v2)) {
                        // Create a Voxel with the computed color
                        geometry::Voxel voxel(Eigen::Vector3i(x, y, z));
                        voxel.color_ = color;
                        voxel_grid->AddVoxel(voxel);
                    }
                }
            }
        }
    }

    return voxel_grid;
}

int main(int argc, char** argv) {
    // Load the mesh from a .obj file
    auto mesh = std::make_shared<geometry::TriangleMesh>();
    if (!io::ReadTriangleMesh("/home/boolmon/Documents/G17 Holo/model/kurumi/kurumi.obj", *mesh)) {
        std::cerr << "Failed to load mesh!" << std::endl;
        return -1;
    }

    // Ensure the mesh has vertex colors loaded
    if (mesh->vertex_colors_.empty()) {
        std::cerr << "Mesh does not have colors!" << std::endl;
        return -1;
    }

    // Set the voxel size
    double voxel_size = 0.05; // Adjust based on desired resolution

    // Voxelize the mesh with colors
    auto voxel_grid = VoxelizeMeshWithColor(mesh, voxel_size);

    // Visualize the result
    visualization::DrawGeometries({voxel_grid}, "Voxelized Model with Color");

    return 0;
}
