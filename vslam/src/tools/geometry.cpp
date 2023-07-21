/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-03 16:06
#
# Filename:		geometry.cpp
#
# Description:
#
************************************************/

#include "tools/geometry.hpp"

namespace vslam {

void Rot3::update(const Eigen::Vector3d &theta) {
  Eigen::Quaterniond dq = Converter::deltaQ(theta);
  rot_ *= dq.toRotationMatrix();
}

void NavState::update(const double dt, const IMUMeasure &imu,
                      const IMUMeasure &imu_prev, const IMUBias &bias,
                      const Eigen::Vector3d &g) {
  Eigen::Vector3d acc0 = imu_prev.head<3>();
  Eigen::Vector3d gyr0 = imu_prev.tail<3>();
  Eigen::Vector3d acc1 = imu.head<3>();
  Eigen::Vector3d gyr1 = imu.tail<3>();
  Eigen::Vector3d ba = bias.head<3>();
  Eigen::Vector3d bg = bias.tail<3>();

  Eigen::Vector3d acc_bar_0 = R_.matrix() * (acc0 - ba) - g;

  Eigen::Vector3d gyr_bar = 0.5 * (gyr0 + gyr1) - bg;
  R_.update(gyr_bar * dt);

  Eigen::Vector3d acc_bar_1 = R_.matrix() * (acc1 - ba) - g;
  Eigen::Vector3d acc_bar = 0.5 * (acc_bar_0 + acc_bar_1);

  p_ += dt * v_ + 0.5 * dt * dt * acc_bar;
  v_ += dt * acc_bar;
}

} // namespace vslam
