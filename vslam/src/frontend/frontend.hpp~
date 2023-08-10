/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-03 09:47
#
# Filename:		frontend.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_FRONTEND_HPP_
#define VSLAM_FRONTEND_HPP_

#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <unordered_map>
//#include <gtsam/navigation/ImuBias.h>
//#include <gtsam/navigation/CombinedImuFactor.h>

#include "dataflow/dataflow.hpp"
#include "frontend/feature_manager.hpp"
#include "frontend/feature_tracker.hpp"
#include "frontend/initial_alignment.hpp"
#include "optimizer/factor_graph_optimizer.hpp"
#include "optimizer/imu_preintegration.hpp"
#include "tools/converter.hpp"
#include "tools/geometry.hpp"
#include "tools/message_print.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {

class FrontEnd {
public:
  FrontEnd(ros::NodeHandle &nh, std::string work_space_path);

  void run();
  void clearState();

private:
  void startTracking();
  void trackImage(double t, const cv::Mat &img0, const cv::Mat &img1);
  void initIMUPose(std::vector<IMUMeasureTime> &imu_interv);
  void processFrame(const double &time, const PointsTrack &feature_frame);
  void preintIMU(const std::vector<IMUMeasureTime> &imu_intv);
  void optimize();
  void updateOptimizedStates();
  void updateLatestStates();
  void predictIMU(double t, Eigen::Vector3d linear_acceleration,
                  Eigen::Vector3d angular_velocity);
  void slideWindow();
  void detectOutliers(std::set<int> &rm_idx);
  double reprojectionError(const NavState &nsi, const NavState &nsj,
                           const Eigen::Isometry3d &Tici,
                           const Eigen::Isometry3d &Ticj, double depth,
                           const Eigen::Vector3d &uvi,
                           const Eigen::Vector3d &uvj);

  bool flag_ = false;

private:
  double cur_time_, prev_time_;
  int track_img_cnt_ = 0;
  int frame_cnt_ = 0;
  bool margin_old_ = false;
  bool init_first_pose_flag_ = false;
  bool is_initialized_flag_ = false;
  bool received_imu_ = false;
  double focal_length_ = 460.0;

  Eigen::Vector3d gravity;

  IMUMeasure imu_meas_0_;
  std::vector<std::vector<double>> dt_buf_;
  std::vector<std::vector<IMUMeasure>> imu_meas_buf_;

  int window_size_ = 10;
  std::vector<double> time_stamps_;
  std::vector<NavState> nav_states_;
  std::vector<IMUBias> imu_bias_;

  double latest_time_;
  NavState latest_nav_state_;
  IMUBias latest_imu_bias_;
  IMUMeasure latest_imu_meas_;

  Eigen::Isometry3d Tic0_calib_, Tic1_calib_;
  std::queue<std::pair<double, PointsTrack>> feature_buf_;

  std::map<double, ImageFrame> image_frames_;
  std::shared_ptr<IMUPreintegration> imu_preint_ptr_;

  std::vector<std::shared_ptr<IMUPreintegration>> imu_preintegrations_;

  std::thread thread_track_;
  std::mutex buff_mutex_;
  std::mutex prop_mutex_;
  ros::Publisher pub_track_image_;

  std::shared_ptr<DataFlow> dataflow_ptr_;
  std::shared_ptr<FeatureTracker> feature_tracker_ptr_;
  std::shared_ptr<FeatureManager> feature_manager_ptr_;
  std::shared_ptr<InitialAlignment> initial_alignment_ptr_;
  std::shared_ptr<FactorGraphOptimizer> graph_optimizer_ptr_;

  std::shared_ptr<MarginalizationInfo> marginalization_info_;
  std::vector<double *> marginalization_param_blocks_;
};
} // namespace vslam

#endif
