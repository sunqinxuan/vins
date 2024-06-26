/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef VSLAM_FEATURE_MANAGER_H
#define VSLAM_FEATURE_MANAGER_H

#include <algorithm>
#include <list>
#include <map>
#include <numeric>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/geometry.hpp"
#include "tools/message_print.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {
class FeaturePerFrame {
public:
  FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    cur_td = td;
    is_stereo = false;
  }
  void rightObservation(const Eigen::Matrix<double, 7, 1> &_point) {
    pointRight.x() = _point(0);
    pointRight.y() = _point(1);
    pointRight.z() = _point(2);
    uvRight.x() = _point(3);
    uvRight.y() = _point(4);
    velocityRight.x() = _point(5);
    velocityRight.y() = _point(6);
    is_stereo = true;
  }
  double cur_td;
  Eigen::Vector3d point, pointRight;
  Eigen::Vector2d uv, uvRight;
  Eigen::Vector2d velocity, velocityRight;
  bool is_stereo;
};

class FeaturePerId {
public:
  const int feature_id;
  int start_frame;
  std::vector<FeaturePerFrame> feature_per_frame;
  int used_num;
  double estimated_depth;
  int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id), start_frame(_start_frame), used_num(0),
        estimated_depth(-1.0), solve_flag(0) {}

  int endFrame();
};

class FeatureManager {
public:
  FeatureManager();
  FeatureManager(const double focal_length, const double min_parallax,
                 const int window_size);

  void clearState();
  int getFeatureCount();
  Eigen::VectorXd getDepthVector();

  // FeatureManager(Matrix3d _Rs[]);

  // void setRic(Matrix3d _ric[]);
  bool addFeatureCheckParallax(int frame_count, const PointsTrack &image,
                               double td);
  void initFramePoseByPnP(int frameCnt, std::vector<NavState> &navStates,
                          const Eigen::Isometry3d &Tic);
  bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                      std::vector<cv::Point2f> &pts2D,
                      std::vector<cv::Point3f> &pts3D);
  void triangulate(std::vector<NavState> &navStates,
                   const Eigen::Isometry3d &Tic0,
                   const Eigen::Isometry3d &Tic1);
  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                        Eigen::Vector3d &point_3d);
  void setDepth(const Eigen::VectorXd &x);
  void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                            Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
  void removeBack();
  void removeFront(int frame_count);
  void removeOutlier(std::set<int> &outlierIndex);
  void removeFailures();
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  getCorresponding(int frame_count_l, int frame_count_r);
  ////void updateDepth(const VectorXd &x);
  // void clearDepth();

  std::list<FeaturePerId> feature;

private:
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  // const Matrix3d *Rs;
  // Matrix3d ric[2];

private:
  int last_track_num;
  double last_average_parallax;
  int new_feature_num;
  int long_track_num;
  double focal_length_, min_parallax_;
  int window_size_;
};
} // namespace vslam
#endif
