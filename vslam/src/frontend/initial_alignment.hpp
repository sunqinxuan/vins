/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-03 09:50
#
# Filename:		initial_alignment.hpp
#
# Description:
#
************************************************/
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#ifndef VSLAM_INITIAL_ALIGNMENT_HPP_
#define VSLAM_INITIAL_ALIGNMENT_HPP_

#include <eigen3/Eigen/Dense>
//#include <gtsam/navigation/CombinedImuFactor.h>
#include <iostream>
#include <map>

//#include <ros/ros.h>
//#include "../factor/imu_factor.h"
//#include "../utility/utility.h"
//#include "../estimator/feature_manager.h"
#include "optimizer/imu_preintegration.hpp"
#include "tools/message_print.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {
class ImageFrame {
public:
  ImageFrame(){};
  ImageFrame(const PointsTrack &_points, double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  PointsTrack points;
  double t;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  // IntegrationBase *pre_integration;
  // std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preint_ptr_;
  std::shared_ptr<IMUPreintegration> imu_preint_ptr_;
  bool is_key_frame;
};

class InitialAlignment {
public:
  void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame,
                          std::vector<IMUBias> &imu_bias);
  //	bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame,
  // Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);

private:
};
} // namespace vslam

#endif
