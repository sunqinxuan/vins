/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-03 16:06
#
# Filename:		geometry.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_TOOLS_GEOMETRY_
#define VSLAM_TOOLS_GEOMETRY_

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>

#include "tools/converter.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {

class Rot3 {
public:
  Rot3() : rot_(Eigen::Matrix3d::Identity()) {}

  Eigen::Matrix3d matrix() const { return rot_; }
  Eigen::Quaterniond quaternion() const { return Eigen::Quaterniond(rot_); }

  void setIdentity() { rot_.setIdentity(); }
  void setRotation(const Eigen::Matrix3d &R) { rot_ = R; }

  void update(const Eigen::Vector3d &theta);

  void swap(Rot3 &other) { rot_.swap(other.rot_); }

private:
  Eigen::Matrix3d rot_;
};

class NavState {
public:
  NavState() : p_(Eigen::Vector3d::Zero()), v_(Eigen::Vector3d::Zero()) {}

  void clearState() {
    R_.setIdentity();
    p_.setZero();
    v_.setZero();
  }

  Eigen::Matrix3d rotation() const { return R_.matrix(); }
  Eigen::Quaterniond quaternion() const { return R_.quaternion(); }
  Eigen::Vector3d position() const { return p_; }
  Eigen::Vector3d velocity() const { return v_; }

  void setRotation(const Eigen::Matrix3d &R) { R_.setRotation(R); }
  void setQuaternion(const Eigen::Quaterniond &q) {
    Eigen::Quaterniond qq = q.normalized();
    R_.setRotation(qq.toRotationMatrix());
  }
  void setPosition(const Eigen::Vector3d &p) { p_ = p; }
  void setVelocity(const Eigen::Vector3d &v) { v_ = v; }

  void update(const double dt, const IMUMeasure &imu,
              const IMUMeasure &imu_prev, const IMUBias &bias,
              const Eigen::Vector3d &g);

  friend std::ostream &operator<<(std::ostream &os, const NavState &state) {
    os << "R: " << std::endl << state.R_.matrix() << std::endl;
    os << "p: " << state.p_.transpose() << std::endl;
    os << "v: " << state.v_.transpose() << std::endl;
    return os;
  }

  void swap(NavState &other) {
    R_.swap(other.R_);
    p_.swap(other.p_);
    v_.swap(other.v_);
  }

private:
  Rot3 R_;
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  //	Eigen::Vector3d a_;
  //	Eigen::Vector3d w_;
};

} // namespace vslam
#endif
