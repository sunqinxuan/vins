/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-13 10:48
#
# Filename:		factor_graph_optimizer.cpp
#
# Description:
#
************************************************/

#include "optimizer/factor_graph_optimizer.hpp"
//#include "global_defination/message_print.hpp"
//#include "tools/tic_toc.hpp"

namespace vslam {

FactorGraphOptimizer::FactorGraphOptimizer(const int num_iter,
                                           const double max_time,
                                           const bool debug)
    : max_num_iterations_(num_iter), max_solver_time_(max_time),
      minimizer_progress_to_stdout_(debug) {
  problem_ = new ceres::Problem;
  loss_function_ = new ceres::HuberLoss(1.0);

  // VINS-Fusion estimator.cpp p1063
  vertices_td_(0) = 0.0;
  problem_->AddParameterBlock(vertices_td_.data(), 1);
  problem_->SetParameterBlockConstant(vertices_td_.data());
}

// Ownership
// Problem by default takes ownership of the cost_function, loss_function,
// local_parameterization, and manifold pointers. These objects remain live for
// the life of the Problem. If the user wishes to keep control over the
// destruction of these objects, then they can do this by setting the
// corresponding enums in the Problem::Options struct.
FactorGraphOptimizer::~FactorGraphOptimizer() { delete problem_; }

bool FactorGraphOptimizer::optimize() {
  if (problem_ == nullptr) {
    ERROR("invalid optimization problem!");
    return false;
  }

  DEBUG("before optimization");
  DEBUG("vertices_nav_state_");
  for (auto &v : vertices_nav_state_) {
    std::cout << v.first << std::endl;
    std::cout << v.second.pose.transpose() << std::endl;
    std::cout << v.second.vel_bias.transpose() << std::endl;
  }
  DEBUG("vertices_camera_ex_");
  for (auto &v : vertices_camera_ex_) {
    std::cout << v.first << "\t" << v.second.pose.transpose() << std::endl;
  }

  for (auto &v : vertices_feature_) {
    std::cout << v.first << "\t" << v.second.depth(0) << std::endl;
  }

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.max_num_iterations = max_num_iterations_;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;
  options.max_solver_time_in_seconds = max_solver_time_;

  ceres::Solve(options, problem_, &summary);

  if (minimizer_progress_to_stdout_)
    std::cout << summary.FullReport() << std::endl;
  DEBUG("Iterations: ", summary.iterations.size());

  DEBUG("after optimization");
  DEBUG("vertices_nav_state_");
  for (auto &v : vertices_nav_state_) {
    std::cout << v.first << std::endl;
    std::cout << v.second.pose.transpose() << std::endl;
    std::cout << v.second.vel_bias.transpose() << std::endl;
  }
  DEBUG("vertices_camera_ex_");
  for (auto &v : vertices_camera_ex_) {
    std::cout << v.first << "\t" << v.second.pose.transpose() << std::endl;
  }

  for (auto &v : vertices_feature_) {
    std::cout << v.first << "\t" << v.second.depth(0) << std::endl;
  }

  return summary.IsSolutionUsable();
}

void FactorGraphOptimizer::addVertexNavState(const int idx,
                                             const NavState &nav_state,
                                             const IMUBias &bias, bool fix) {
  // vertices_nav_state_.insert(std::pair<int, VertexParamNavState>(
  // idx, VertexParamNavState(nav_state, bias, fix)));
  vertices_nav_state_.insert(
      std::make_pair(idx, VertexParamNavState(nav_state, bias, fix)));

  ceres::LocalParameterization *local_param = new PoseLocalParameterization();
  problem_->AddParameterBlock(vertices_nav_state_[idx].pose.data(), 7,
                              local_param);
  problem_->AddParameterBlock(vertices_nav_state_[idx].vel_bias.data(), 9);

  /// DEBUG(idx, "\t", vertices_nav_state_[idx].pose.transpose());
  // INFO(idx, "\t", vertices_nav_state_[idx].vel_bias.transpose());

  if (fix) {
    problem_->SetParameterBlockConstant(vertices_nav_state_[idx].pose.data());
    problem_->SetParameterBlockConstant(
        vertices_nav_state_[idx].vel_bias.data());
  }
}

void FactorGraphOptimizer::addVertexCameraEx(const int idx,
                                             const Eigen::Isometry3d &Tic,
                                             bool fix) {
  vertices_camera_ex_.insert(
      std::pair<int, VertexParamCameraEx>(idx, VertexParamCameraEx(Tic, fix)));

  ceres::LocalParameterization *local_param = new PoseLocalParameterization();
  problem_->AddParameterBlock(vertices_camera_ex_[idx].pose.data(), 7,
                              local_param);
  if (fix)
    problem_->SetParameterBlockConstant(vertices_camera_ex_[idx].pose.data());
}

void FactorGraphOptimizer::addVertexFeature(const int idx, const double val) {
  vertices_feature_.insert(
      std::pair<int, VertexParamFeature>(idx, VertexParamFeature(val)));
}

bool FactorGraphOptimizer::getNavStateBias(const int idx, NavState &nav_state,
                                           IMUBias &bias,
                                           const Eigen::Matrix3d &dR0,
                                           const Eigen::Vector3d &P0) const {
  if (!getNavStateBias(idx, nav_state, bias))
    return false;
  auto it = vertices_nav_state_.begin();
  Eigen::Vector3d p0(it->second.pose(0), it->second.pose(1),
                     it->second.pose(2));
  nav_state.setRotation(dR0 * nav_state.rotation());
  nav_state.setPosition(dR0 * (nav_state.position() - p0) + P0);
  nav_state.setVelocity(dR0 * nav_state.velocity());
  return true;
}
bool FactorGraphOptimizer::getNavStateBias(const int idx, NavState &nav_state,
                                           IMUBias &bias) const {
  auto it = vertices_nav_state_.find(idx);
  if (it == vertices_nav_state_.end())
    return false;
  Eigen::Quaterniond q(it->second.pose(6), it->second.pose(3),
                       it->second.pose(4), it->second.pose(5));
  Eigen::Vector3d p(it->second.pose(0), it->second.pose(1), it->second.pose(2));
  Eigen::Vector3d v(it->second.vel_bias(0), it->second.vel_bias(1),
                    it->second.vel_bias(2));
  nav_state.setRotation(q.normalized().toRotationMatrix());
  nav_state.setPosition(p);
  nav_state.setVelocity(v);
  bias = it->second.vel_bias.tail<6>();
  return true;
}

bool FactorGraphOptimizer::getNavState(const int idx,
                                       NavState &nav_state) const {
  auto it = vertices_nav_state_.find(idx);
  if (it == vertices_nav_state_.end())
    return false;
  Eigen::Quaterniond q(it->second.pose(6), it->second.pose(3),
                       it->second.pose(4), it->second.pose(5));
  Eigen::Vector3d p(it->second.pose(0), it->second.pose(1), it->second.pose(2));
  Eigen::Vector3d v(it->second.vel_bias(0), it->second.vel_bias(1),
                    it->second.vel_bias(2));
  nav_state.setRotation(q.normalized().toRotationMatrix());
  nav_state.setPosition(p);
  nav_state.setVelocity(v);
  return true;
}

bool FactorGraphOptimizer::getCameraEx(const int idx,
                                       Eigen::Isometry3d &Tic) const {
  auto it = vertices_camera_ex_.find(idx);
  if (it == vertices_camera_ex_.end())
    return false;
  Eigen::Quaterniond q(it->second.pose(6), it->second.pose(3),
                       it->second.pose(4), it->second.pose(5));
  Eigen::Vector3d p(it->second.pose(0), it->second.pose(1), it->second.pose(2));
  Tic.linear() = q.normalized().toRotationMatrix();
  Tic.translation() = p;
  return true;
}

bool FactorGraphOptimizer::getFeature(const int idx, double &val) const {
  auto it = vertices_feature_.find(idx);
  if (it == vertices_feature_.end())
    return false;
  val = it->second.depth(0);
  return true;
}

// estimator.cpp
// line 1068
// TODO
// bool FactorGraphOptimizer::addFactorMargin(
//    const std::shared_ptr<MarginalizationInfo> &margin_info) {}

bool FactorGraphOptimizer::addFactorIMU(
    const int idx_pre, const int idx,
    const std::shared_ptr<IMUPreintegration> &imu_preint) {
  if (imu_preint->getSumDt() > 10.0) {
    WARNING("[FactorGraph][addIMUFactor] invalid imu preint sum time");
    return false;
  }
  auto it = vertices_nav_state_.find(idx);
  if (it == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx);
    return false;
  }
  auto it_pre = vertices_nav_state_.find(idx_pre);
  if (it_pre == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx_pre);
    return false;
  }
  IMUFactor *imu_factor = new IMUFactor(imu_preint);
  problem_->AddResidualBlock(imu_factor, NULL,
                             vertices_nav_state_[idx_pre].pose.data(),
                             vertices_nav_state_[idx_pre].vel_bias.data(),
                             vertices_nav_state_[idx].pose.data(),
                             vertices_nav_state_[idx].vel_bias.data());
  return true;
}

bool FactorGraphOptimizer::addFactorProject12(
    const int f_idx, const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector2d &vel_i, const Eigen::Vector2d &vel_j,
    const double td_i, const double td_j) {
  auto it = vertices_feature_.find(f_idx);
  if (it == vertices_feature_.end()) {
    WARNING("[FactorGraph][addFactorProject12] not found vertex with id ",
            f_idx);
    return false;
  }

  ProjectionOneFrameTwoCamFactor *factor = new ProjectionOneFrameTwoCamFactor(
      pts_i, pts_j, vel_i, vel_j, td_i, td_j);
  problem_->AddResidualBlock(
      factor, loss_function_, vertices_camera_ex_[0].pose.data(),
      vertices_camera_ex_[1].pose.data(), vertices_feature_[f_idx].depth.data(),
      vertices_td_.data());
  return true;
}

bool FactorGraphOptimizer::addFactorProject21(
    const int idx_i, const int idx_j, const int f_idx,
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector2d &vel_i, const Eigen::Vector2d &vel_j,
    const double td_i, const double td_j) {
  auto it = vertices_feature_.find(f_idx);
  if (it == vertices_feature_.end()) {
    WARNING("[FactorGraph][addFactorProject12] not found vertex with id ",
            f_idx);
    return false;
  }
  auto it_i = vertices_nav_state_.find(idx_i);
  if (it_i == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx_i);
    return false;
  }
  auto it_j = vertices_nav_state_.find(idx_j);
  if (it_j == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx_j);
    return false;
  }

  ProjectionTwoFrameOneCamFactor *factor = new ProjectionTwoFrameOneCamFactor(
      pts_i, pts_j, vel_i, vel_j, td_i, td_j);
  problem_->AddResidualBlock(
      factor, loss_function_, vertices_nav_state_[idx_i].pose.data(),
      vertices_nav_state_[idx_j].pose.data(),
      vertices_camera_ex_[0].pose.data(), vertices_feature_[f_idx].depth.data(),
      vertices_td_.data());
  return true;
}

bool FactorGraphOptimizer::addFactorProject22(
    const int idx_i, const int idx_j, const int f_idx,
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector2d &vel_i, const Eigen::Vector2d &vel_j,
    const double td_i, const double td_j) {
  auto it = vertices_feature_.find(f_idx);
  if (it == vertices_feature_.end()) {
    WARNING("[FactorGraph][addFactorProject12] not found vertex with id ",
            f_idx);
    return false;
  }
  auto it_i = vertices_nav_state_.find(idx_i);
  if (it_i == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx_i);
    return false;
  }
  auto it_j = vertices_nav_state_.find(idx_j);
  if (it_j == vertices_nav_state_.end()) {
    WARNING("[FactorGraph][addIMUFactor] not found vertex with id ", idx_j);
    return false;
  }

  ProjectionTwoFrameTwoCamFactor *factor = new ProjectionTwoFrameTwoCamFactor(
      pts_i, pts_j, vel_i, vel_j, td_i, td_j);
  problem_->AddResidualBlock(
      factor, loss_function_, vertices_nav_state_[idx_i].pose.data(),
      vertices_nav_state_[idx_j].pose.data(),
      vertices_camera_ex_[0].pose.data(), vertices_camera_ex_[1].pose.data(),
      vertices_feature_[f_idx].depth.data(), vertices_td_.data());
  return true;
}

} // namespace vslam
