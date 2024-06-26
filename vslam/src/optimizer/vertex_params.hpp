/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-11-11 10:47
#
# Filename:		vertex_pose.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_OPTIMIZER_VERTEX_HPP_
#define VSLAM_OPTIMIZER_VERTEX_HPP_

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

#include "tools/geometry.hpp"
//#include "optimizer/marginalization_factor.hpp"

namespace vslam {
// using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector1d = Eigen::Matrix<double, 1, 1>;

class VertexParamNavState {
public:
  VertexParamNavState() {
    pose.setZero();
    pose(6) = 1.0;
    vel_bias.setZero();
  }
  void setValues(const NavState &nav_state, const IMUBias &bias) {
    pose(0) = nav_state.position().x();
    pose(1) = nav_state.position().y();
    pose(2) = nav_state.position().z();
    pose(3) = nav_state.quaternion().x();
    pose(4) = nav_state.quaternion().y();
    pose(5) = nav_state.quaternion().z();
    pose(6) = nav_state.quaternion().w();

    vel_bias(0) = nav_state.velocity().x();
    vel_bias(1) = nav_state.velocity().y();
    vel_bias(2) = nav_state.velocity().z();
    vel_bias.tail<6>() = bias;
  }
  void setValues(const NavState &nav_state) {
    pose(0) = nav_state.position().x();
    pose(1) = nav_state.position().y();
    pose(2) = nav_state.position().z();
    pose(3) = nav_state.quaternion().x();
    pose(4) = nav_state.quaternion().y();
    pose(5) = nav_state.quaternion().z();
    pose(6) = nav_state.quaternion().w();

    // vel_bias(0) = nav_state.velocity().x();
    // vel_bias(1) = nav_state.velocity().y();
    // vel_bias(2) = nav_state.velocity().z();
    // vel_bias.tail<6>() = bias;
  }

  Vector7d pose;
  Vector9d vel_bias;
  // bool fixed;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class VertexParamCameraEx {
public:
  VertexParamCameraEx() {
    pose.setZero();
    pose(6) = 1.0;
  }
  void setValues(const Eigen::Isometry3d &Tic) {
    pose(0) = Tic.translation().x();
    pose(1) = Tic.translation().y();
    pose(2) = Tic.translation().z();
    Eigen::Quaterniond q(Tic.linear());
    pose(3) = q.x();
    pose(4) = q.y();
    pose(5) = q.z();
    pose(6) = q.w();
  }

  Vector7d pose;
  // bool fixed;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class VertexParamFeature {
public:
  VertexParamFeature() : depth(0.0) {}
  void setValues(const double d) { depth(0) = d; }

  Vector1d depth;
  // bool fixed;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// class VertexParamMargin {
// public:
//  VertexParamMargin() = delete;
//  // VertexParamMargin() {}
//};

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

// class FactorParamMargin
//{
//	public:
//		FactorParamMargin()=delete;
//		FactorParamMargin(std::shared_ptr<MarginalizationInfo>
// m):margin_info_ptr(p) {}
//
//		std::shared_ptr<MarginalizationInfo> margin_info_ptr;
//};
//
// class FactorParamIMU
//{
//};

// class FactorParamProject12
//{
//	public:
//	FactorParamProject12() =delete;
//	FactorParamProject12(const Eigen::Vector3d &_pts_i, const
// Eigen::Vector3d &_pts_j,
//    				   const Eigen::Vector2d &_vel_i, const
//    Eigen::Vector2d
//    &_vel_j, 				   const double _td_i, const double
//    _td_j)
//		: pts_i(_pts_i),pts_j(_pts_j),
//		vel_i(_vel_i),vel_j(_vel_j),
//		td_i(_td_i),td_j(_td_j) {}
//
//	Eigen::Vector3d pts_i,pts_j;
//	 Eigen::Vector2d vel_i, vel_j;
//	 double td_i, td_j;
//};
//
// class FactorParamProject21
//{
//	public:
//	FactorParamProject21() =delete;
//	FactorParamProject21(const Eigen::Vector3d &_pts_i, const
// Eigen::Vector3d &_pts_j,
//    				   const Eigen::Vector2d &_vel_i, const
//    Eigen::Vector2d
//    &_vel_j, 				   const double _td_i, const double
//    _td_j)
//		: pts_i(_pts_i),pts_j(_pts_j),
//		vel_i(_vel_i),vel_j(_vel_j),
//		td_i(_td_i),td_j(_td_j) {}
//
//	Eigen::Vector3d pts_i,pts_j;
//	 Eigen::Vector2d vel_i, vel_j;
//	 double td_i, td_j;
//};
//
// class FactorParamProject22
//{
//	public:
//	FactorParamProject22() =delete;
//	FactorParamProject22(const Eigen::Vector3d &_pts_i, const
// Eigen::Vector3d &_pts_j,
//    				   const Eigen::Vector2d &_vel_i, const
//    Eigen::Vector2d
//    &_vel_j, 				   const double _td_i, const double
//    _td_j)
//		: pts_i(_pts_i),pts_j(_pts_j),
//		vel_i(_vel_i),vel_j(_vel_j),
//		td_i(_td_i),td_j(_td_j) {}
//
//	Eigen::Vector3d pts_i,pts_j;
//	 Eigen::Vector2d vel_i, vel_j;
//	 double td_i, td_j;
//};

// class SE3Parameterization : public ceres::LocalParameterization
//{
// public:
//  SE3Parameterization()
//  {
//  }
//  virtual ~SE3Parameterization()
//  {
//  }
//
//  virtual bool Plus(const double *x, const double *delta, double
//  *x_plus_delta) const
//  {
//    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
//    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);
//
//    Sophus::SE3 T = Sophus::SE3::exp(lie);
//    Sophus::SE3 delta_T = Sophus::SE3::exp(delta_lie);
//
//    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();
//
//    for (int i = 0; i < 6; ++i) {
//      x_plus_delta[i] = x_plus_delta_lie(i, 0);
//    }
//
//    return true;
//  }
//  virtual bool ComputeJacobian(const double *x, double *jacobian) const
//  {
//    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
//    return true;
//  }
//  virtual int GlobalSize() const
//  {
//    return 6;
//  }
//  virtual int LocalSize() const
//  {
//    return 6;
//  }
//};

} // namespace vslam
#endif
