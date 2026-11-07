#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <mutex>
#include <deque>
#include <cmath>

#include "tight_lio/imu_integrator.hpp"
#include "tight_lio/point_deskewer.hpp"
#include "tight_lio/icp_matcher.hpp"
#include "tight_lio/iekf_estimator.hpp"

namespace tight_lio {

class TightLioNode : public rclcpp::Node {
public:
    TightLioNode()
        : Node("tight_lio_node") {
        declareParameters();
        loadParameters();
        initComponents();
        createSubscribersPublishers();
        RCLCPP_INFO(get_logger(), "TightLioNode initialised. Waiting for data...");
    }

private:
    void declareParameters(){
        declare_parameter("T_imu_lidar_translation", std::vector<double>{0,0,0});
        declare_parameter("T_imu_lidar_rotation",    std::vector<double>{1,0,0,0});
        declare_parameter("imu_acc_noise",       0.01);
        declare_parameter("imu_gyro_noise",      0.001);
        declare_parameter("imu_acc_bias_noise",  0.0001);
        declare_parameter("imu_gyro_bias_noise", 0.00001);
        declare_parameter("imu_init_samples",    200);
        declare_parameter("use_msg_orientation_init", false);
        declare_parameter("initial_yaw_offset_deg", 0.0);
        declare_parameter("trajectory_yaw_offset_deg", 0.0);
        declare_parameter("iekf_max_iterations",         5);
        declare_parameter("iekf_max_residuals",         400);
        declare_parameter("iekf_startup_scans",         15);
        declare_parameter("iekf_residual_gate_m",       0.5);
        declare_parameter("iekf_max_translation_step_m", 0.15);
        declare_parameter("iekf_max_velocity_step_mps",  0.50);
        declare_parameter("iekf_max_rotation_step_rad",  0.05);
        declare_parameter("max_position_jump_m",         2.0);
        declare_parameter("max_velocity_jump_mps",       3.0);
        declare_parameter("max_abs_velocity_mps",        40.0);
        declare_parameter("iekf_convergence_threshold",  0.001);
        declare_parameter("icp_num_neighbors",           20);
        declare_parameter("icp_plane_valid_threshold",   0.5);
        declare_parameter("icp_max_correspondence_dist", 1.0);
        declare_parameter("icp_min_points_for_update",   50);
        declare_parameter("map_voxel_size",              0.3);
        declare_parameter("map_publish_voxel_size",      0.5);
        declare_parameter("enable_map_publish",          false);
        declare_parameter("safe_mode",                   true);
        declare_parameter("max_points_per_scan",         4000);
        declare_parameter("lidar_topic",  "/ouster64/points");
        declare_parameter("imu_topic",    "/imu/data");
        declare_parameter("world_frame",  "world");
        declare_parameter("imu_frame",    "imu");
        declare_parameter("lidar_frame",  "ouster");
        declare_parameter("iekf_min_points_for_update", 50);
        declare_parameter("measurement_noise_std", 0.05);
    }

    void loadParameters() {
        auto t = get_parameter("T_imu_lidar_translation").as_double_array();
        auto r = get_parameter("T_imu_lidar_rotation").as_double_array();

        T_imu_lidar_ = Eigen::Isometry3d::Identity();
        T_imu_lidar_.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
        T_imu_lidar_.linear() = Eigen::Quaterniond(r[0],r[1],r[2],r[3]).toRotationMatrix();
    
        imu_noise_.acc_noise        = get_parameter("imu_acc_noise").as_double();
        imu_noise_.gyro_noise       = get_parameter("imu_gyro_noise").as_double();
        imu_noise_.acc_bias_noise   = get_parameter("imu_acc_bias_noise").as_double();
        imu_noise_.gyro_bias_noise  = get_parameter("imu_gyro_bias_noise").as_double();
        imu_init_samples_           = get_parameter("imu_init_samples").as_int();
        use_msg_orientation_init_   = get_parameter("use_msg_orientation_init").as_bool();
        initial_yaw_offset_deg_     = get_parameter("initial_yaw_offset_deg").as_double();
        trajectory_yaw_offset_deg_  = get_parameter("trajectory_yaw_offset_deg").as_double();

        iekf_params_.max_iterations        = get_parameter("iekf_max_iterations").as_int();
        iekf_params_.max_residuals         = get_parameter("iekf_max_residuals").as_int();
        iekf_startup_scans_                = get_parameter("iekf_startup_scans").as_int();
        iekf_params_.residual_gate_m       = get_parameter("iekf_residual_gate_m").as_double();
        iekf_params_.max_translation_step_m = get_parameter("iekf_max_translation_step_m").as_double();
        iekf_params_.max_velocity_step_mps  = get_parameter("iekf_max_velocity_step_mps").as_double();
        iekf_params_.max_rotation_step_rad  = get_parameter("iekf_max_rotation_step_rad").as_double();
        max_position_jump_m_               = get_parameter("max_position_jump_m").as_double();
        max_velocity_jump_mps_             = get_parameter("max_velocity_jump_mps").as_double();
        max_abs_velocity_mps_              = get_parameter("max_abs_velocity_mps").as_double();
        iekf_params_.convergence_threshold = get_parameter("iekf_convergence_threshold").as_double();
        iekf_params_.min_points_for_update = get_parameter("iekf_min_points_for_update").as_int();
        icp_params_.num_neighbors           = get_parameter("icp_num_neighbors").as_int();
        icp_params_.plane_valid_threshold   = get_parameter("icp_plane_valid_threshold").as_double();
        icp_params_.max_correspondence_dist = get_parameter("icp_max_correspondence_dist").as_double();
        icp_params_.min_points_for_update   = get_parameter("icp_min_points_for_update").as_int();
        icp_params_.voxel_size              = get_parameter("map_voxel_size").as_double();

        iekf_params_.min_points_for_update = get_parameter("iekf_min_points_for_update").as_int();
        iekf_params_.measurement_noise_std = get_parameter("measurement_noise_std").as_double();


        map_publish_voxel_size_ = get_parameter("map_publish_voxel_size").as_double();
        enable_map_publish_ = get_parameter("enable_map_publish").as_bool();
        safe_mode_ = get_parameter("safe_mode").as_bool();
        max_points_per_scan_ = get_parameter("max_points_per_scan").as_int();
        world_frame_    = get_parameter("world_frame").as_string();
        imu_frame_      = get_parameter("imu_frame").as_string();
        lidar_frame_    = get_parameter("lidar_frame").as_string();
        lidar_topic_    = get_parameter("lidar_topic").as_string();
        imu_topic_      = get_parameter("imu_topic").as_string();

        if (safe_mode_) {
            RCLCPP_WARN(get_logger(), "safe_mode=true: running lightweight LiDAR update for runtime stability.");
        }
    }

    void initComponents() {
        imu_integrator_ = std::make_unique<ImuIntegrator>(imu_noise_);
        point_deskewer_ = std::make_unique<PointDeskewer>(T_imu_lidar_);
        icp_matcher_    = std::make_unique<IcpMatcher>(icp_params_);
        iekf_estimator_ = std::make_unique<IekfEstimator>(iekf_params_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publishStaticTf();
    }

    void publishStaticTf() {
        Eigen::Isometry3d T_imu_to_lidar = T_imu_lidar_.inverse();

        geometry_msgs::msg::TransformStamped tf_static =
            tf2::eigenToTransform(T_imu_to_lidar);
        tf_static.header.stamp = now();
        tf_static.header.frame_id = imu_frame_;
        tf_static.child_frame_id = lidar_frame_;

        static_tf_broadcaster_->sendTransform(tf_static);
        RCLCPP_INFO(get_logger(),
                    "Published static TF %s -> %s",
                    imu_frame_.c_str(),
                    lidar_frame_.c_str());
    }

    void createSubscribersPublishers(){
        auto qos = rclcpp::SensorDataQoS();

        sub_imu_   = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos,
            std::bind(&TightLioNode::imuCallback, this, std::placeholders::_1));

        sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, qos,
            std::bind(&TightLioNode::lidarCallback, this, std::placeholders::_1));

        pub_odom_  = create_publisher<nav_msgs::msg::Odometry>("tight_lio/odom", 10);
        pub_path_  = create_publisher<nav_msgs::msg::Path>("tight_lio/path", 10);
        pub_map_   = create_publisher<sensor_msgs::msg::PointCloud2>(
            "tight_lio/map", rclcpp::QoS(1).transient_local());
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);

        if (!imu_initialised_) {
            Eigen::Vector3d acc(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
            Eigen::Vector3d gyro(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);

            acc_sum_  += acc;
            gyro_sum_ += gyro;
            imu_init_count_++;

            if (imu_init_count_ < imu_init_samples_) {
                if (imu_init_count_ == 1) {
                    RCLCPP_INFO(get_logger(),
                                "Collecting %d IMU samples for initial alignment...",
                                imu_init_samples_);
                }
                return;
            }

            const Eigen::Vector3d acc_mean = acc_sum_ / static_cast<double>(imu_init_count_);
            const Eigen::Vector3d gyro_mean = gyro_sum_ / static_cast<double>(imu_init_count_);

            Eigen::Quaterniond q_init = Eigen::Quaterniond::Identity();
            if (use_msg_orientation_init_) {
                Eigen::Quaterniond q_msg(msg->orientation.w,
                                         msg->orientation.x,
                                         msg->orientation.y,
                                         msg->orientation.z);
                if (q_msg.coeffs().allFinite() && q_msg.norm() > 1e-6) {
                    q_msg.normalize();
                    q_init = q_msg;
                }
            }

            if (!use_msg_orientation_init_) {
                if (acc_mean.norm() > 1e-3) {
                    q_init = Eigen::Quaterniond::FromTwoVectors(
                        acc_mean.normalized(), Eigen::Vector3d::UnitZ());
                    q_init.normalize();
                }
            }

            if (std::abs(initial_yaw_offset_deg_) > 1e-6) {
                const double yaw_rad = initial_yaw_offset_deg_ * M_PI / 180.0;
                Eigen::Quaterniond q_yaw(
                    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
                q_init = q_yaw * q_init;
                q_init.normalize();
            }

            static const Eigen::Vector3d gravity_world(0.0, 0.0, 9.81);
            Eigen::Vector3d expected_acc_body = q_init.inverse() * gravity_world;

            double t0 = rclcpp::Time(msg->header.stamp).seconds();
            NavState s;
            s.timestamp = t0;
            s.p.setZero();
            s.v.setZero();
            s.q  = q_init;
            s.ba = acc_mean - expected_acc_body;
            s.bg = gyro_mean;

            Eigen::Matrix<double,18,18> P = Eigen::Matrix<double,18,18>::Zero();
            P.block<3,3>(0,0)   = 1e-3 * Eigen::Matrix3d::Identity();
            P.block<3,3>(3,3)   = 1e-2 * Eigen::Matrix3d::Identity();
            P.block<3,3>(6,6)   = 1e-3 * Eigen::Matrix3d::Identity();
            P.block<3,3>(9,9)   = 1e-2 * Eigen::Matrix3d::Identity();
            P.block<3,3>(12,12) = 1e-3 * Eigen::Matrix3d::Identity();

            imu_integrator_->setState(s, P);
            imu_initialised_ = true;
            RCLCPP_INFO(get_logger(),
                        "IMU initialised at t=%.3f using %d samples (|acc_mean|=%.3f, |gyro_bias|=%.4f, yaw_offset=%.1f deg)",
                        t0,
                        imu_init_count_,
                        acc_mean.norm(),
                        s.bg.norm(),
                        initial_yaw_offset_deg_);
        }

        ImuData imu;
        imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();
        imu.acc  = Eigen::Vector3d(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
        imu.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);
        imu_integrator_->addImu(imu);
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        if (!imu_initialised_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for first IMU message...");
            return;
        }
        std::lock_guard<std::mutex> lk(mutex_);

        double t_scan = rclcpp::Time(msg->header.stamp).seconds();

        auto raw_points = extractTimestampedPoints(msg, t_scan);
        if(raw_points.empty()) return;

        double t_start = t_scan;
        double t_end = t_start + 0.1;
        for (const auto& p : raw_points) {
            t_start = std::min(t_start, p.timestamp);
            t_end   = std::max(t_end,   p.timestamp);
        }

        NavState                    propagated_state;
        Eigen::Matrix<double,18,18> propagated_cov;
        bool ok = imu_integrator_->propagateTo(t_end, propagated_state, propagated_cov);
        if (!ok) {
        RCLCPP_WARN(get_logger(), "IMU propagation failed — skipping scan");
        return;
        }

        auto trajectory = imu_integrator_->getPoseTrajectory(t_start, t_end);
        auto deskewed = point_deskewer_->deskew(raw_points, trajectory, t_start);
        if (deskewed.empty()) return;

        auto deskewed_used = capPointsPerScan(deskewed);
        if (deskewed_used.empty()) return;

        Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
        T_world_imu.translation() = propagated_state.p;
        T_world_imu.linear() = propagated_state.q.toRotationMatrix();
        Eigen::Isometry3d T_world_lidar = T_world_imu * T_imu_lidar_.inverse();

        NavState updated_state;
        Eigen::Matrix<double,18,18> updated_cov;

        if (safe_mode_) {
            updated_state = propagated_state;
            updated_cov = propagated_cov;
        } else {
            std::vector<PlaneResidual> residuals;
            if (!icp_matcher_->mapIsEmpty()) {
                residuals = icp_matcher_->computeResiduals(deskewed_used, T_world_lidar);
            }
            ++processed_scans_;
            if (processed_scans_ < iekf_startup_scans_) {
                updated_state = propagated_state;
                updated_cov = propagated_cov;
            } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "IEKF update: %zu valid residuals, map_size=%d",
                std::count_if(residuals.begin(), residuals.end(),
                            [](const PlaneResidual& r){ return r.valid; }),
                icp_matcher_->mapSize());
            iekf_estimator_->update(propagated_state, propagated_cov, residuals, T_imu_lidar_, updated_state, updated_cov);
            }
            // ── NaN guard ─────────────────────────────────────────────────────────────
            const double position_jump = (updated_state.p - propagated_state.p).norm();
            const double velocity_jump = (updated_state.v - propagated_state.v).norm();
            if (!updated_state.p.allFinite() ||
                !updated_state.v.allFinite() ||
                !updated_state.q.coeffs().allFinite() ||
                position_jump > max_position_jump_m_ ||
                velocity_jump > max_velocity_jump_mps_ ||
                updated_state.v.norm() > max_abs_velocity_mps_) {
                RCLCPP_ERROR(get_logger(),
                    "State diverged (dp=%.2f dv=%.2f v=%.2f) — reverting",
                    position_jump, velocity_jump, updated_state.v.norm());
                updated_state = propagated_state;
                updated_cov   = propagated_cov;
                // Also reset P to prevent runaway covariance
                updated_cov.setZero();
                updated_cov.block<3,3>(0,0)   = 0.01 * Eigen::Matrix3d::Identity();
                updated_cov.block<3,3>(3,3)   = 0.01 * Eigen::Matrix3d::Identity();
                updated_cov.block<3,3>(6,6)   = 0.01 * Eigen::Matrix3d::Identity();
                updated_cov.block<3,3>(9,9)   = 1e-4 * Eigen::Matrix3d::Identity();
                updated_cov.block<3,3>(12,12) = 1e-4 * Eigen::Matrix3d::Identity();
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "State: p=[%.2f, %.2f, %.2f] v=[%.2f, %.2f, %.2f]",
            updated_state.p.x(), updated_state.p.y(), updated_state.p.z(),
            updated_state.v.x(), updated_state.v.y(), updated_state.v.z());
        
        imu_integrator_->setState(updated_state, updated_cov);

        Eigen::Isometry3d T_world_imu_updated = Eigen::Isometry3d::Identity();
        T_world_imu_updated.translation() = updated_state.p;
        T_world_imu_updated.linear()      = updated_state.q.toRotationMatrix();
        Eigen::Isometry3d T_world_lidar_updated = T_world_imu_updated * T_imu_lidar_.inverse();

        std::vector<Eigen::Vector3d> points_world;
        points_world.reserve(deskewed_used.size());
        for (const auto& p : deskewed_used)
        points_world.push_back(T_world_lidar_updated * p);

        icp_matcher_->insertPoints(points_world);

        publishOdom(updated_state, msg->header.stamp);
        publishTf(updated_state, msg->header.stamp);

        static int scan_count = 0;
        if(enable_map_publish_ && ++scan_count % 10 == 0)
            publishMap(msg->header.stamp);
    }

    std::vector<Eigen::Vector3d> capPointsPerScan(const std::vector<Eigen::Vector3d>& points) const {
        if (max_points_per_scan_ <= 0 ||
            static_cast<int>(points.size()) <= max_points_per_scan_) {
            return points;
        }

        std::vector<Eigen::Vector3d> sampled;
        sampled.reserve(max_points_per_scan_);

        const size_t step =
            (points.size() + static_cast<size_t>(max_points_per_scan_) - 1) /
            static_cast<size_t>(max_points_per_scan_);

        for (size_t i = 0; i < points.size() &&
                          static_cast<int>(sampled.size()) < max_points_per_scan_;
             i += step) {
            sampled.push_back(points[i]);
        }

        return sampled;
    }

    std::vector<TimestampedPoint> extractTimestampedPoints( 
      const sensor_msgs::msg::PointCloud2::SharedPtr& msg, double scan_start_time) { 
      
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        bool has_time_field = false;
        int  time_offset    = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "t" || field.name == "time" || field.name == "timestamp") {
                has_time_field = true;
                time_offset    = static_cast<int>(field.offset);
                break;
            }
        }
        std::vector<TimestampedPoint> pts;
        pts.reserve(cloud->size());

        for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        TimestampedPoint tp;
        tp.x = p.x; tp.y = p.y; tp.z = p.z;
        tp.intensity = p.intensity;

        if (has_time_field) {
            // Ouster 't' field: nanoseconds relative to scan start
            uint32_t t_ns;
            std::memcpy(&t_ns,
                        &msg->data[i * msg->point_step + time_offset],
                        sizeof(uint32_t));
            tp.timestamp = scan_start_time + t_ns * 1e-9;
        } else {
            // Uniformly distribute across 100ms rotation
            tp.timestamp = scan_start_time +
                        static_cast<double>(i) / cloud->size() * 0.1;
        }
        pts.push_back(tp);
        }
        return pts;
    }

    void publishOdom(const NavState& s, const rclcpp::Time& stamp) {
        Eigen::AngleAxisd yaw_rot(trajectory_yaw_offset_deg_ * M_PI / 180.0,
                                  Eigen::Vector3d::UnitZ());

        Eigen::Vector3d p_pub = yaw_rot * s.p;
        Eigen::Vector3d v_pub = yaw_rot * s.v;
        Eigen::Quaterniond q_pub(yaw_rot);
        q_pub = q_pub * s.q;
        q_pub.normalize();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = world_frame_;
        odom.child_frame_id  = imu_frame_;

        odom.pose.pose.position.x = p_pub.x();
        odom.pose.pose.position.y = p_pub.y();
        odom.pose.pose.position.z = p_pub.z();
        odom.pose.pose.orientation.w = q_pub.w();
        odom.pose.pose.orientation.x = q_pub.x();
        odom.pose.pose.orientation.y = q_pub.y();
        odom.pose.pose.orientation.z = q_pub.z();

        odom.twist.twist.linear.x = v_pub.x();
        odom.twist.twist.linear.y = v_pub.y();
        odom.twist.twist.linear.z = v_pub.z();

        pub_odom_->publish(odom);

        // Append to path
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose   = odom.pose.pose;
        path_.header = odom.header;
        path_.poses.push_back(ps);
        pub_path_->publish(path_);
    }

    void publishTf(const NavState& s, const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = world_frame_;
        tf_msg.child_frame_id  = imu_frame_;

        tf_msg.transform.translation.x = -s.p.x();
        tf_msg.transform.translation.y = -s.p.y();
        tf_msg.transform.translation.z = s.p.z();
        tf_msg.transform.rotation.w    = s.q.w();
        tf_msg.transform.rotation.x    = s.q.x();
        tf_msg.transform.rotation.y    = s.q.y();
        tf_msg.transform.rotation.z    = s.q.z();

        tf_broadcaster_->sendTransform(tf_msg);
    }


    void publishMap(const rclcpp::Time& stamp) {
        auto map_pts = icp_matcher_->getMapPoints();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(map_pts.size());
        for (const auto& p : map_pts)
        cloud->emplace_back(static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z()));

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(map_publish_voxel_size_,
                    map_publish_voxel_size_,
                    map_publish_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*filtered, out_msg);
        out_msg.header.stamp    = stamp;
        out_msg.header.frame_id = world_frame_;
        pub_map_->publish(out_msg);

    }

    std::mutex mutex_;
    bool imu_initialised_ = false;
    int  imu_init_samples_ = 200;
    int  imu_init_count_ = 0;
    bool use_msg_orientation_init_ = false;
    double initial_yaw_offset_deg_ = 0.0;
    double trajectory_yaw_offset_deg_ = 0.0;
    Eigen::Vector3d acc_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum_ = Eigen::Vector3d::Zero();
    Eigen::Isometry3d         T_imu_lidar_;
    ImuIntegrator::NoiseParams imu_noise_;
    IekfEstimator::Params      iekf_params_;
    IcpMatcher::Params         icp_params_;
    double                     map_publish_voxel_size_;
    bool                       enable_map_publish_ = false;
    bool                       safe_mode_ = false;
    int                        max_points_per_scan_ = 4000;
    double                     max_position_jump_m_ = 2.0;
    double                     max_velocity_jump_mps_ = 3.0;
    double                     max_abs_velocity_mps_ = 40.0;
    int                        iekf_startup_scans_ = 15;
    int                        processed_scans_ = 0;
    std::string                world_frame_, imu_frame_, lidar_frame_;
    std::string                lidar_topic_, imu_topic_;

    std::unique_ptr<ImuIntegrator>  imu_integrator_;
    std::unique_ptr<PointDeskewer>  point_deskewer_;
    std::unique_ptr<IcpMatcher>     icp_matcher_;
    std::unique_ptr<IekfEstimator>  iekf_estimator_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_lidar_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_map_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>            static_tf_broadcaster_;

    nav_msgs::msg::Path path_;

};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tight_lio::TightLioNode>());
    rclcpp::shutdown();
    return 0;
}

