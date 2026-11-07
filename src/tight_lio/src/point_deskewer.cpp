#include "tight_lio/point_deskewer.hpp"
#include <algorithm>
#include <cmath>

namespace tight_lio {

PointDeskewer::PointDeskewer(const Eigen::Isometry3d& T_imu_lidar): T_imu_lidar_(T_imu_lidar),  T_lidar_imu_(T_imu_lidar_.inverse()) {}

std::vector<Eigen::Vector3d> PointDeskewer::deskew(
    const std::vector<TimestampedPoint>& points,
    const std::vector<StampedPose>& trajectory,
    double t_start){
    std::vector<Eigen::Vector3d> deskewed;
    deskewed.reserve(points.size());

    if(trajectory.empty()){
        for(const auto& pt:points)
            deskewed.emplace_back(pt.x, pt.y, pt.z);
        return deskewed;
    }

    Eigen::Isometry3d T_w_imu_start = interpolatePose(trajectory, t_start);

    for(const auto& pt: points){
        Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);

        if(!std::isfinite(p_lidar.x()) || !std::isfinite(p_lidar.y()) || !std::isfinite(p_lidar.z())){
            continue;
        }

        Eigen::Isometry3d T_w_imu_ti = interpolatePose(trajectory, pt.timestamp);
        Eigen::Isometry3d T_rel = T_w_imu_start.inverse() * T_w_imu_ti;

        Eigen::Vector3d p_imu_ti    = T_imu_lidar_ * p_lidar;
        Eigen::Vector3d p_imu_start = T_rel.inverse() * p_imu_ti;
        Eigen::Vector3d p_lidar_deskewed = T_lidar_imu_ * p_imu_start;
        deskewed.push_back(p_lidar_deskewed);
    }
    return deskewed;
}

Eigen::Isometry3d PointDeskewer::interpolatePose(const std::vector<StampedPose>& traj, double t) const {
    if(traj.empty()) return Eigen::Isometry3d::Identity();

    if(t <= traj.front().timestamp){
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = traj.front().position;
        T.linear() = traj.front().orientation.toRotationMatrix();
        return T;
    }
    if (t>= traj.back().timestamp){
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = traj.back().position;
        T.linear() = traj.back().orientation.toRotationMatrix();
        return T;
    }

    auto it = std::lower_bound(traj.begin(), traj.end(), t, [](const StampedPose& sp, double t){
        return sp.timestamp < t;
    });
    const StampedPose& sp1 = *it;
    const StampedPose& sp0 = *(it - 1);

    double alpha = (t - sp0.timestamp) / (sp1.timestamp - sp0.timestamp);

    Eigen::Vector3d p = sp0.position + alpha * (sp1.position - sp0.position);
    Eigen::Quaterniond q = sp0.orientation.slerp(alpha, sp1.orientation);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = p;
    T.linear()      = q.toRotationMatrix();
    return T;

} 
}