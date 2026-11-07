#include "tight_lio/icp_matcher.hpp"
#include <Eigen/SVD>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>

namespace tight_lio {

IcpMatcher::IcpMatcher(const Params& params) : params_(params) {
    ikd_tree_ = std::make_unique<KD_TREE<pcl::PointXYZ>>(0.3, 0.6, params_.voxel_size);
}

void IcpMatcher::insertPoints(const std::vector<Eigen::Vector3d>& points_world) {
  if (points_world.empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points_world.size());
  for (const auto& p : points_world) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(p.x());
    pt.y = static_cast<float>(p.y());
    pt.z = static_cast<float>(p.z());
    cloud->push_back(pt);
  }

  if (map_size_ == 0) {
    ikd_tree_->Build(cloud->points);
    map_size_ = static_cast<int>(cloud->size());
    return;
  }

  ikd_tree_->Add_Points(cloud->points, true);  // true = downsample
  map_size_ += static_cast<int>(cloud->size());
}

std::vector<PlaneResidual> IcpMatcher::computeResiduals(
    const std::vector<Eigen::Vector3d>& points_lidar,
    const Eigen::Isometry3d& T_world_lidar) {

  std::vector<PlaneResidual> residuals;
  residuals.reserve(points_lidar.size());

  if (mapIsEmpty()) return residuals;

  for (const auto& p_lidar : points_lidar) {
    PlaneResidual res;
    res.valid        = false;
    res.point_lidar  = p_lidar;
    const Eigen::Vector3d query_world = T_world_lidar * p_lidar;
    res.point_world  = query_world;

    // ── kNN search in ikd-Tree ───────────────────────────────────────────────
    pcl::PointXYZ query;
    query.x = static_cast<float>(query_world.x());
    query.y = static_cast<float>(query_world.y());
    query.z = static_cast<float>(query_world.z());

    std::vector<float>          sq_dists;
    pcl::PointCloud<pcl::PointXYZ> neighbors;

    // ikd-Tree kNN search
    ikd_tree_->Nearest_Search(query, params_.num_neighbors,
                             neighbors.points, sq_dists);

    if (static_cast<int>(neighbors.size()) < params_.num_neighbors) {
      residuals.push_back(res);
      continue;
    }

    // Reject if nearest neighbour is too far
    if (sq_dists[0] > params_.max_correspondence_dist *
                       params_.max_correspondence_dist) {
      residuals.push_back(res);
      continue;
    }

    // ── Plane fitting via SVD ────────────────────────────────────────────────
    std::vector<Eigen::Vector3d> nb_pts;
    nb_pts.reserve(neighbors.size());
    for (const auto& nb : neighbors.points)
      nb_pts.emplace_back(nb.x, nb.y, nb.z);

    Eigen::Vector3d normal, centroid;
    if (!fitPlane(nb_pts, normal, centroid)) {
      residuals.push_back(res);
      continue;
    }

    // ── Residual ─────────────────────────────────────────────────────────────
    res.plane_normal = normal;
    res.point_world  = centroid;
    res.residual     = normal.dot(query_world - centroid);
    res.valid        = true;
    residuals.push_back(res);
  }

  return residuals;
}


bool IcpMatcher::fitPlane(const std::vector<Eigen::Vector3d>& pts,
                            Eigen::Vector3d& normal,
                            Eigen::Vector3d& centroid) const {
  int n = static_cast<int>(pts.size());
  centroid = Eigen::Vector3d::Zero();
  for (const auto& p : pts) centroid += p;
  centroid /= n;

  Eigen::MatrixXd A(n, 3);
  for (int i = 0; i < n; ++i)
    A.row(i) = (pts[i] - centroid).transpose();

  // Economy SVD: A = U * S * V^T
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::VectorXd s = svd.singularValues();

  // s is sorted descending. Smallest value is s(n-1).
  // Validity: ratio of smallest to second-smallest eigenvalue (approx s^2)
  double ratio = s(s.size()-1) / (s(s.size()-2) + 1e-10);
  if (ratio > params_.plane_valid_threshold) return false;

  // Normal = column of V corresponding to smallest singular value
  normal = svd.matrixV().col(s.size()-1);
  return true;
}

std::vector<Eigen::Vector3d> IcpMatcher::getMapPoints() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ikd_tree_->flatten(ikd_tree_->Root_Node, cloud->points,
                        NOT_RECORD);  // ikd-Tree flatten API
    std::vector<Eigen::Vector3d> out;
    out.reserve(cloud->size());
    for (const auto& p : cloud->points)
        out.emplace_back(p.x, p.y, p.z);
    return out;
}

}