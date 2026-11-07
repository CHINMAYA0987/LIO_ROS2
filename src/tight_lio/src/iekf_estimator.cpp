#include "tight_lio/iekf_estimator.hpp"
#include "tight_lio/imu_integrator.hpp"  // for skew(), so3Exp()
#include <Eigen/Dense>
#include <algorithm>

namespace tight_lio {

IekfEstimator::IekfEstimator(const Params& params) : params_(params) {}

void IekfEstimator::update(
    const NavState& x_prop,
    const Eigen::Matrix<double,18,18>& P_prop,
    const std::vector<PlaneResidual>& residuals,
    const Eigen::Isometry3d& T_imu_lidar,
    NavState& x_updated,
    Eigen::Matrix<double,18,18>& P_updated) {

    Eigen::Matrix<double,18,18> P_safe = P_prop;
    for (int i = 0; i < 18; ++i)
        P_safe(i,i) = std::min(P_safe(i,i), 1.0);

    // Collect valid residual references
    std::vector<const PlaneResidual*> valid_residuals;
    valid_residuals.reserve(residuals.size());
    for (const auto& r : residuals) {
        if (r.valid) valid_residuals.push_back(&r);
    }
    const int num_valid = static_cast<int>(valid_residuals.size());
    
    if (num_valid < params_.min_points_for_update) {
        // Not enough correspondences — skip update, propagation only
        x_updated = x_prop;
        P_updated = P_safe;
        return;
    }

    const int num_used = std::min(num_valid, params_.max_residuals);
    std::vector<const PlaneResidual*> selected_residuals;
    selected_residuals.reserve(num_used);
    if (num_used == num_valid) {
        selected_residuals = valid_residuals;
    } else {
        for (int i = 0; i < num_used; ++i) {
            const int idx = static_cast<int>(
                static_cast<long long>(i) * num_valid / num_used);
            selected_residuals.push_back(valid_residuals[idx]);
        }
    }

    const double R_noise = params_.measurement_noise_std *
                            params_.measurement_noise_std;
    NavState x_iter = x_prop;

    for(int iter = 0; iter < params_.max_iterations; ++iter){
        Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
        T_world_imu.translation() = x_iter.p;
        T_world_imu.linear() = x_iter.q.toRotationMatrix();

        Eigen::Isometry3d T_world_lidar = T_world_imu * T_imu_lidar.inverse();

        Eigen::MatrixXd H(num_used, 18);
        Eigen::VectorXd z(num_used);
        int row = 0;

        for(const auto* res_ptr : selected_residuals){
            const auto& res = *res_ptr;

            Eigen::Vector3d p_world = T_world_lidar * res.point_lidar;
            PlaneResidual res_iter = res;
            res_iter.residual = res.plane_normal.dot(p_world - res.point_world);
            if (std::abs(res_iter.residual) > params_.residual_gate_m) {
                continue;
            }
            H.row(row) = buildHrow(res_iter, x_iter.q, T_imu_lidar);
            z(row) = -res_iter.residual;
            ++row;     
        }

        if (row < params_.min_points_for_update) {
            x_updated = x_prop;
            P_updated = P_safe;
            return;
        }

        H.conservativeResize(row, 18);
        z.conservativeResize(row);

        Eigen::MatrixXd R_mat = R_noise * Eigen::MatrixXd::Identity(row, row);
        Eigen::MatrixXd S = H * P_safe * H.transpose() + R_mat;

        Eigen::MatrixXd K = P_safe * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(row, row));
        Eigen::Matrix<double,18,1> delta_iter = Eigen::Matrix<double,18,1>::Zero();
        delta_iter.segment<3>(0)  = x_iter.p  - x_prop.p;
        delta_iter.segment<3>(3)  = x_iter.v  - x_prop.v;

        Eigen::Quaterniond dq = x_iter.q * x_prop.q.inverse();
        delta_iter.segment<3>(6)  = 2.0 * dq.vec();  // small-angle approximation
        delta_iter.segment<3>(9)  = x_iter.ba - x_prop.ba;
        delta_iter.segment<3>(12) = x_iter.bg - x_prop.bg;

        Eigen::Matrix<double,18,1> dx = K * (z - H * delta_iter);
        if (dx.segment<3>(0).norm() > params_.max_translation_step_m)
            dx.segment<3>(0) *= params_.max_translation_step_m / dx.segment<3>(0).norm();
        if (dx.segment<3>(3).norm() > params_.max_velocity_step_mps)
            dx.segment<3>(3) *= params_.max_velocity_step_mps / dx.segment<3>(3).norm();
        if (dx.segment<3>(6).norm() > params_.max_rotation_step_rad)
            dx.segment<3>(6) *= params_.max_rotation_step_rad / dx.segment<3>(6).norm();

        x_iter = boxplus(x_iter, dx);
        if(dx.norm() < params_.convergence_threshold){
            break;
        }
    }

    Eigen::Isometry3d T_world_imu_final = Eigen::Isometry3d::Identity();
    T_world_imu_final.translation() = x_iter.p;
    T_world_imu_final.linear()      = x_iter.q.toRotationMatrix();
    Eigen::Isometry3d T_world_lidar_final = T_world_imu_final * T_imu_lidar.inverse();

    Eigen::MatrixXd H_final(num_used, 18);
    int row = 0;
    for (const auto* res_ptr : selected_residuals) {
        const auto& res = *res_ptr;
        PlaneResidual res_iter = res;
        Eigen::Vector3d p_world = T_world_lidar_final * res.point_lidar;
        res_iter.residual = res.plane_normal.dot(p_world - res.point_world);
        if (std::abs(res_iter.residual) > params_.residual_gate_m) {
            continue;
        }
        H_final.row(row) = buildHrow(res_iter, x_iter.q, T_imu_lidar);
        ++row;
    }

    if (row < params_.min_points_for_update) {
        x_updated = x_prop;
        P_updated = P_safe;
        return;
    }

    H_final.conservativeResize(row, 18);

    Eigen::MatrixXd R_mat  = R_noise * Eigen::MatrixXd::Identity(row, row);
    Eigen::MatrixXd S_final = H_final * P_safe * H_final.transpose() + R_mat;
    Eigen::MatrixXd K_final = P_safe * H_final.transpose() *
                                S_final.ldlt().solve(
                                Eigen::MatrixXd::Identity(row, row));

    // Joseph form: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    Eigen::Matrix<double,18,18> IKH = Eigen::Matrix<double,18,18>::Identity() -
                                        K_final * H_final;
    P_updated = IKH * P_prop * IKH.transpose() +
                K_final * R_mat * K_final.transpose();

    x_updated = x_iter;
        
}

Eigen::Matrix<double,1,18> IekfEstimator::buildHrow(
    const PlaneResidual& res,
    const Eigen::Quaterniond& q,
    const Eigen::Isometry3d& T_imu_lidar) const {

    Eigen::Matrix<double,1,18> H = Eigen::Matrix<double,1,18>::Zero();
    Eigen::Matrix3d R_wi = q.toRotationMatrix();
    Eigen::Vector3d v = T_imu_lidar * res.point_lidar;

    const Eigen::Vector3d& n = res.plane_normal;
    H.block<1,3>(0,0) = n.transpose();
    H.block<1,3>(0,6) = -n.transpose() * R_wi * skew(v);
    return H;
}

NavState IekfEstimator::boxplus(const NavState& x, const Eigen::Matrix<double,18,1>& dx) const {
    NavState xn = x;
    xn.p += dx.segment<3>(0);
    xn.v += dx.segment<3>(3);
    xn.q = x.q * so3Exp(dx.segment<3>(6));
    xn.ba += dx.segment<3>(9);
    xn.bg += dx.segment<3>(12);
    return xn;
}

}