/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-13 10:48
#
# Filename:		factor_graph_optimizer.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_FACTOR_GRAPH_OPTIMIZER_HPP_
#define VSLAM_FACTOR_GRAPH_OPTIMIZER_HPP_

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "optimizer/camera_projection_factor.hpp"
#include "optimizer/imu_factor.hpp"
#include "optimizer/imu_preintegration.hpp"
#include "optimizer/vertex_params.hpp"
#include "tools/geometry.hpp"
#include "tools/message_print.hpp"

namespace vslam {
class FactorGraphOptimizer {
public:
  using MapVertexNavState = std::map<int, VertexParamNavState>;
  using MapVertexCameraEx = std::map<int, VertexParamCameraEx>;
  using MapVertexFeature = std::map<int, VertexParamFeature>;

  FactorGraphOptimizer() = delete;
  FactorGraphOptimizer(const int num_iter, const double max_time,
                       const bool debug);
  ~FactorGraphOptimizer();

  bool optimize();

  void addVertexNavState(const int idx,const NavState &nav_state, const IMUBias &bias,
                         bool fix = false);
  void addVertexCameraEx(const int idx, const Eigen::Isometry3d &Tic,
                         bool fix = false);
void addVertexFeature(const int idx, const double val);

  //	bool addFactorMargin();
  bool addFactorIMU(const int idx_pre, const int idx,
                    const std::shared_ptr<IMUPreintegration> &imu_preint);

  bool addFactorProject12(const int f_idx, const Eigen::Vector3d &pts_i,
                          const Eigen::Vector3d &pts_j,
                          const Eigen::Vector2d &vel_i,
                          const Eigen::Vector2d &vel_j, const double td_i,
                          const double td_j);
bool addFactorProject21(
    const int idx_i, const int idx_j, const int f_idx,
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector2d &vel_i, const Eigen::Vector2d &vel_j,
    const double td_i, const double td_j) ;
bool addFactorProject22(
    const int idx_i, const int idx_j, const int f_idx,
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector2d &vel_i, const Eigen::Vector2d &vel_j,
    const double td_i, const double td_j) ;


private:
private:
  int max_num_iterations_ = 10;
  double max_solver_time_ = 0.05;
  bool minimizer_progress_to_stdout_ = false;

  ceres::Problem *problem_;
  ceres::LossFunction *loss_function_;

  MapVertexNavState vertices_nav_state_;
  MapVertexCameraEx vertices_camera_ex_;
  MapVertexFeature vertices_feature_;
  Eigen::Matrix<double, 1, 1> vertices_td_;
};
} // namespace vslam
#endif