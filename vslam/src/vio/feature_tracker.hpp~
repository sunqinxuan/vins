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

#ifndef VSLAM_FEATURE_TRACKER_HPP_
#define VSLAM_FEATURE_TRACKER_HPP_

#include <execinfo.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "tools/message_print.hpp"
#include "tools/tic_toc.hpp"
#include "tools/type_redefine.hpp"
//#include "../estimator/parameters.h"
//#include "../utility/tic_toc.h"

namespace vslam {

// using namespace std;
using namespace camodocal;
// using namespace Eigen;

// bool inBorder(const cv::Point2f &pt);
void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

class FeatureTracker {
public:
  FeatureTracker(int max_cnt, int min_dist, double F_thres);
  void readCameraIntrinsics(const std::string &cam0_file,
                            const std::string &cam1_file);
  bool trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1,
                  PointsTrack &featureFrame);
  cv::Mat getTrackImage();

private:
  void setMask();
  // void showUndistortion(const string &name);
  // void rejectWithF();
  // void undistortedPoints();
  std::vector<cv::Point2f> undistortedPts(std::vector<cv::Point2f> &pts,
                                          camodocal::CameraPtr cam);
  std::vector<cv::Point2f> ptsVelocity(std::vector<int> &ids,
                                       std::vector<cv::Point2f> &pts,
                                       std::map<int, cv::Point2f> &cur_id_pts,
                                       std::map<int, cv::Point2f> &prev_id_pts);
  // void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
  //                  vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
  void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                 std::vector<int> &curLeftIds,
                 std::vector<cv::Point2f> &curLeftPts,
                 std::vector<cv::Point2f> &curRightPts,
                 std::map<int, cv::Point2f> &prevLeftPtsMap);
  // void setPrediction(map<int, Eigen::Vector3d> &predictPts);
  double distance(cv::Point2f &pt1, cv::Point2f &pt2);
  // void removeOutliers(set<int> &removePtsIds);
  bool inBorder(const cv::Point2f &pt);

private:
  int MAX_CNT = 150, MIN_DIST = 30;
  double F_THRESHOLD = 1.0;

  double cur_time, prev_time;
  int n_id;
  bool hasPrediction;
  int row, col;
  cv::Mat imTrack;
  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img;
  camodocal::CameraPtr camera0, camera1;
  std::vector<cv::Point2f> n_pts;
  std::vector<cv::Point2f> predict_pts;
  std::vector<cv::Point2f> predict_pts_debug;
  std::vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
  std::vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
  std::vector<cv::Point2f> pts_velocity, right_pts_velocity;
  std::vector<int> ids, ids_right;
  std::vector<int> track_cnt;
  std::map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
  std::map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
  std::map<int, cv::Point2f> prevLeftPtsMap;
};

} // namespace vslam

#endif
