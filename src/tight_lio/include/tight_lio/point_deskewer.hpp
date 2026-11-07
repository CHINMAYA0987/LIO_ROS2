#pragma once
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tight_lio/imu_integrator.hpp"

namespace tight_lio {
struct TimestampedPoint{
    float x, y, z;
    double timestamp;
    float intensity;
};

class PointDeskewer {
public:
    PointDeskewer(const Eigen::Isometry3d& T_imu_lidar);

    std::vector<Eigen::Vector3d> deskew(
      const std::vector<TimestampedPoint>& points,
      const std::vector<StampedPose>& trajectory,
      double t_start);

private:

    Eigen::Isometry3d interpolatePose(const std::vector<StampedPose>& traj,
                                     double t) const;

    Eigen::Isometry3d T_imu_lidar_;
    Eigen::Isometry3d T_lidar_imu_;

};

}