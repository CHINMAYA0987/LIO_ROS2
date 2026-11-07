#pragma once
#include <Eigen/Dense>
#include <vector>
#include <Eigen/Geometry>

#include "ikd_Tree.h"

namespace tight_lio {
struct PlaneResidual {
    Eigen::Vector3d point_world;
    Eigen::Vector3d point_lidar;
    Eigen::Vector3d plane_normal;
    double residual;
    bool valid;
};

// Icp matcher - maintains an ikd tree map in world frame and for a given deskewed
// point cloud + current pose estimate, finds point-to-plane residuals

class IcpMatcher {
public:
    struct Params {
        int  num_neighbors = 5; 
        double plane_valid_threshold = 0.1;
        double max_correspondence_dist = 1.0;
        int min_points_for_update = 10;
        double voxel_size = 0.3;
    };

    explicit IcpMatcher(const Params& params);

    void insertPoints(const std::vector<Eigen::Vector3d>& points_world);

    std::vector<PlaneResidual> computeResiduals(const std::vector<Eigen::Vector3d>& points_lidar, 
                                        const Eigen::Isometry3d& T_world_lidar);

    bool mapIsEmpty() const {return map_size_ == 0;}
    int mapSize() const {return map_size_;}

    std::vector<Eigen::Vector3d> getMapPoints();

private:
    bool fitPlane(const std::vector<Eigen::Vector3d>& neighbors,
                Eigen::Vector3d& normal, Eigen::Vector3d& centroid) const;
    Params params_;
    std::unique_ptr<KD_TREE<pcl::PointXYZ>>   ikd_tree_;
    int map_size_ = 0;

};
}