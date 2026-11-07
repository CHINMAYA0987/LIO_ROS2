#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "tight_lio/imu_integrator.hpp"
#include "tight_lio/icp_matcher.hpp"

namespace tight_lio {
class IekfEstimator {
public:
    struct Params {
        int    max_iterations          = 5;
        int    max_residuals           = 400;
        int    min_points_for_update   = 80;
        double convergence_threshold   = 0.001;
        double measurement_noise_std   = 0.05;
        double residual_gate_m         = 0.5;
        double max_translation_step_m  = 0.15;
        double max_velocity_step_mps   = 0.50;
        double max_rotation_step_rad   = 0.05;
    };
    explicit IekfEstimator(const Params& params);

    void update(const NavState& x_prop,
              const Eigen::Matrix<double,18,18>& P_prop,
              const std::vector<PlaneResidual>& residuals,
              const Eigen::Isometry3d& T_imu_lidar,
              NavState& x_updated,
              Eigen::Matrix<double,18,18>& P_updated);

private:
    Eigen::Matrix<double,1,18> buildHrow(
      const PlaneResidual& res,
      const Eigen::Quaterniond& q,
      const Eigen::Isometry3d& T_imu_lidar) const;

    NavState boxplus(const NavState& x, const Eigen::Matrix<double,18,1>& dx) const;
    Params params_;
};

}