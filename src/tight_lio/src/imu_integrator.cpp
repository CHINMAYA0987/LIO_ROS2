#include "tight_lio/imu_integrator.hpp"
#include <algorithm>
#include <stdexcept>

namespace tight_lio {

// ── Constructor ────────────────────────────────────────────────────────────────
ImuIntegrator::ImuIntegrator(const NoiseParams& noise)
    : noise_(noise) {
  // Initialise state to zero / identity
  state_.timestamp = 0.0;
  state_.p.setZero();
  state_.v.setZero();
  state_.q = Eigen::Quaterniond::Identity();
  state_.ba.setZero();
  state_.bg.setZero();

  P_.setZero();
  // Small initial uncertainty on biases
  P_.block<3,3>(9,9)  = 1e-4 * Eigen::Matrix3d::Identity();
  P_.block<3,3>(12,12) = 1e-4 * Eigen::Matrix3d::Identity();
}

// ── setState ───────────────────────────────────────────────────────────────────
void ImuIntegrator::setState(const NavState& s,
                              const Eigen::Matrix<double,18,18>& P) {
  state_ = s;
  P_     = P;
}

// ── addImu ─────────────────────────────────────────────────────────────────────
void ImuIntegrator::addImu(const ImuData& imu) {
  // Keep buffer sorted by timestamp
  auto it = std::lower_bound(buf_.begin(), buf_.end(), imu,
    [](const ImuData& a, const ImuData& b){ return a.timestamp < b.timestamp; });
  buf_.insert(it, imu);
}

// ── propagateTo ────────────────────────────────────────────────────────────────
bool ImuIntegrator::propagateTo(double target_time,
                                 NavState& state,
                                 Eigen::Matrix<double,18,18>& P) {
  if (buf_.size() < 2) return false;

  double t_current = state_.timestamp;

  // Find IMU samples that bracket [t_current, target_time]
  while (buf_.size() >= 2 && buf_[0].timestamp <= t_current) {
    buf_.pop_front();  // discard samples before current time
  }
  if (buf_.empty()) return false;

  // Integrate up to target_time using available IMU pairs
  for (size_t i = 0; i + 1 < buf_.size(); ++i) {
    const ImuData& imu0 = buf_[i];
    const ImuData& imu1 = buf_[i + 1];

    if (imu0.timestamp >= target_time) break;

    // Create a clipped second sample if it overshoots target_time
    ImuData imu1_clipped = imu1;
    if (imu1.timestamp > target_time) {
      double alpha = (target_time - imu0.timestamp) /
                     (imu1.timestamp - imu0.timestamp);
      imu1_clipped.timestamp = target_time;
      imu1_clipped.acc  = imu0.acc  + alpha * (imu1.acc  - imu0.acc);
      imu1_clipped.gyro = imu0.gyro + alpha * (imu1.gyro - imu0.gyro);
    }

    integrateMidpoint(imu0, imu1_clipped, state_, P_);

    // Store pose in trajectory buffer for deskewing
    StampedPose sp;
    sp.timestamp   = state_.timestamp;
    sp.position    = state_.p;
    sp.orientation = state_.q;
    traj_.push_back(sp);

    if (state_.timestamp >= target_time) break;
  }

  state = state_;
  P     = P_;
  return true;
}

// ── getPoseTrajectory ──────────────────────────────────────────────────────────
std::vector<StampedPose> ImuIntegrator::getPoseTrajectory(
    double t_start, double t_end) {

  std::vector<StampedPose> result;
  for (const auto& sp : traj_) {
    if (sp.timestamp >= t_start && sp.timestamp <= t_end)
      result.push_back(sp);
  }

  // Remove trajectory entries older than t_start
  traj_.erase(std::remove_if(traj_.begin(), traj_.end(),
    [t_start](const StampedPose& sp){ return sp.timestamp < t_start; }),
    traj_.end());

  return result;
}

// ── integrateMidpoint ─────────────────────────────────────────────────────────
//
// Midpoint (RK2) integration of IMU kinematics with bias correction.
//
//   p_{k+1} = p_k + v_k*dt + 0.5*a_mid*dt^2
//   v_{k+1} = v_k + a_mid*dt
//   q_{k+1} = q_k ⊗ Exp(omega_mid * dt)
//
// where a_mid, omega_mid are midpoint averages of the two IMU samples,
// corrected for bias.
//
void ImuIntegrator::integrateMidpoint(const ImuData& imu0,
                                       const ImuData& imu1,
                                       NavState& s,
                                       Eigen::Matrix<double,18,18>& P) {
  double dt = imu1.timestamp - imu0.timestamp;
  if (dt <= 0.0 || dt > 0.5) return;  // sanity check

  Eigen::Matrix3d R = s.q.toRotationMatrix();

  // ── Midpoint IMU readings (bias-corrected) ──────────────────────────────────
  Eigen::Vector3d acc0  = imu0.acc  - s.ba;
  Eigen::Vector3d acc1  = imu1.acc  - s.ba;
  Eigen::Vector3d gyro0 = imu0.gyro - s.bg;
  Eigen::Vector3d gyro1 = imu1.gyro - s.bg;

  Eigen::Vector3d acc_mid  = 0.5 * (acc0 + acc1);
  Eigen::Vector3d gyro_mid = 0.5 * (gyro0 + gyro1);

  static const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  // ── State integration ───────────────────────────────────────────────────────
  Eigen::Vector3d a_world = R * acc_mid + gravity;

  s.p += s.v * dt + 0.5 * a_world * dt * dt;
  s.v += a_world * dt;
  s.q  = s.q * so3Exp(gyro_mid * dt);
  s.q.normalize();
  s.timestamp = imu1.timestamp;

  // ── Covariance propagation: P = F*P*F^T + Q ─────────────────────────────────
  Eigen::Matrix<double,18,18> F = buildF({imu1.timestamp, acc_mid, gyro_mid},
                                          dt, s);
  Eigen::Matrix<double,18,18> Qd = buildQd(dt, s.q);

  P = F * P * F.transpose() + Qd;
  // Clamp diagonal to prevent runaway covariance
  for (int i = 0; i < 18; ++i)
      P(i,i) = std::min(P(i,i), 1e6);
}

// ── buildF ─────────────────────────────────────────────────────────────────────
//
// Discrete-time state transition matrix for the ESKF error state.
// Linearized around current nominal state.
//
//  δp_{k+1} ≈ δp_k + δv_k*dt
//  δv_{k+1} ≈ δv_k - R*[a_m]*δθ*dt - R*δba*dt
//  δθ_{k+1} ≈ (I - [ω_m]*dt)*δθ - δbg*dt
//  δba_{k+1} ≈ δba_k
//  δbg_{k+1} ≈ δbg_k
//
Eigen::Matrix<double,18,18> ImuIntegrator::buildF(
    const ImuData& imu_mid, double dt, const NavState& s) const {

  Eigen::Matrix<double,18,18> F = Eigen::Matrix<double,18,18>::Identity();
  Eigen::Matrix3d R = s.q.toRotationMatrix();
  Eigen::Vector3d a_hat = imu_mid.acc  - s.ba;
  Eigen::Vector3d w_hat = imu_mid.gyro - s.bg;

  // δp / δv
  F.block<3,3>(0,3)  =  Eigen::Matrix3d::Identity() * dt;
  // δv / δθ
  F.block<3,3>(3,6)  = -R * skew(a_hat) * dt;
  // δv / δba
  F.block<3,3>(3,9)  = -R * dt;
  // δθ / δθ
  F.block<3,3>(6,6)  =  Eigen::Matrix3d::Identity() - skew(w_hat) * dt;
  // δθ / δbg
  F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity() * dt;
  // biases: identity (random walk)

  return F;
}

// ── buildQd ────────────────────────────────────────────────────────────────────
//
// Discrete process noise covariance.
// Approximate as continuous Q integrated over dt.
//
Eigen::Matrix<double,18,18> ImuIntegrator::buildQd(
    double dt, const Eigen::Quaterniond& q) const {

  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix<double,18,18> Qd = Eigen::Matrix<double,18,18>::Zero();

  double sa2 = noise_.acc_noise  * noise_.acc_noise;
  double sg2 = noise_.gyro_noise * noise_.gyro_noise;
  double sba2 = noise_.acc_bias_noise  * noise_.acc_bias_noise;
  double sbg2 = noise_.gyro_bias_noise * noise_.gyro_bias_noise;

  // Velocity noise (from accelerometer white noise)
  Qd.block<3,3>(3,3)   = R * (sa2 * dt) * Eigen::Matrix3d::Identity() * R.transpose();
  // Rotation noise (from gyro white noise)
  Qd.block<3,3>(6,6)   = sg2 * dt * Eigen::Matrix3d::Identity();
  // Bias random walk
  Qd.block<3,3>(9,9)   = sba2 * dt * Eigen::Matrix3d::Identity();
  Qd.block<3,3>(12,12) = sbg2 * dt * Eigen::Matrix3d::Identity();

  return Qd;
}

}  // namespace tight_lio