#pragma once
#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tight_lio {

struct ImuData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct StampedPose{
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct NavState {
    double timestamp;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    NavState() : timestamp(0.0),
             p(Eigen::Vector3d::Zero()),
             v(Eigen::Vector3d::Zero()),
             q(Eigen::Quaterniond::Identity()),
             ba(Eigen::Vector3d::Zero()),
             bg(Eigen::Vector3d::Zero()) {}
};


class ImuIntegrator {
public:
    struct NoiseParams {
        double acc_noise = 0.01;
        double gyro_noise = 0.001;
        double acc_bias_noise = 0.0001;
        double gyro_bias_noise = 0.00001;
    };

    explicit ImuIntegrator(const NoiseParams& noise_params);

    void addImu(const ImuData& imu);

    bool propagateTo(double target_time, NavState& state, Eigen::Matrix<double, 18, 18>& P);

    std::vector<StampedPose> getPoseTrajectory(double t_start, double t_end);

    const NavState& state() const {return state_;}

    void setState(const NavState& s, const Eigen::Matrix<double, 18, 18>& P);

private:
    void integrateMidpoint(const ImuData& imu0, const ImuData& imu1, NavState& s, Eigen::Matrix<double, 18, 18>& P);
    Eigen::Matrix<double,18,18> buildQd(double dt, const Eigen::Quaterniond& q) const;
    Eigen::Matrix<double,18,18> buildF(const ImuData& imu_mid, double dt, const NavState& s) const;

    NoiseParams noise_;
    NavState state_;
    Eigen::Matrix<double, 18, 18> P_;
    std::deque<ImuData> buf_;
    std::vector<StampedPose> traj_;

};

inline Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi) {
  double angle = phi.norm();
  if (angle < 1e-10) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, phi / angle));
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<  0,    -v.z(),  v.y(),
        v.z(),  0,    -v.x(),
       -v.y(),  v.x(),  0;
  return S;
}


}
