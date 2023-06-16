/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-15 09:24
#
# Filename: frontend.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_FRONTEND_HPP_
#define VSLAM_FRONTEND_HPP_

//#include "models/cloud_filter/cloud_filter_interface.hpp"
//#include "models/cloud_filter/cloud_filter_thread.hpp"
//#include "models/graph_optimizer/interface_graph_optimizer.hpp"
////#include
///"models/line_feature_extraction/line_feature_extraction_interface.hpp"
//#include "models/registration/registration_interface.hpp"
//#include "sensor_data/cloud_data.hpp"
//#include "sensor_data/ins_data.hpp"
//#include "sensor_data/key_frame.hpp"
//#include "sensor_data/line_feature.hpp"
//#include "sensor_data/pose_data.hpp"
//
//#include "global_defination/message_print.hpp"
//#include "tools/convert_matrix.hpp"
//#include "tools/tic_toc.hpp"
//#include <pcl/kdtree/kdtree_flann.h>
//#include <Eigen/Dense>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
//#include <yaml-cpp/yaml.h>

#include "dataflow/dataflow.hpp"
#include "frontend/feature_tracker.hpp"
#include "frontend/feature_manager.hpp"
#include "tools/message_print.hpp"

namespace vslam {
class FrontEnd {
  // public:
  // struct Frame {
  //  unsigned int id = 0;
  //  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  //  float score = 0.0;
  //  float yaw = 0.0;
  //  CloudData cloud_data;
  //  Eigen::Matrix<float, 6, 6> covariance =
  //      Eigen::Matrix<float, 6, 6>::Identity() * 1e-6;
  //
  //  float weight = 1.0;
  //  PoseData preint_pose;
  //};
  // using FrameDeque = std::deque<Frame>;

public:
  FrontEnd(ros::NodeHandle &nh, std::string work_space_path);

  void run();
  void clearState();

private:
  void startTracking();
  void trackImage(double t, const cv::Mat &img0, const cv::Mat &img1);
  void pubTrackImage(const cv::Mat &imgTrack, const double t);

private:
  double cur_time_, prev_time_;
  int track_img_cnt_ = 0;
	Eigen::Isometry3d Tic0_calib, Tic1_calib;
  std::queue<std::pair<
      double,
      std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>>
      feature_buf_;

  std::thread thread_track_;
  std::mutex buff_mutex_;
  // std::mutex proc_mutex_;
  ros::Publisher pub_track_image_;

  std::shared_ptr<DataFlow> dataflow_ptr_;
  std::shared_ptr<FeatureTracker> feature_tracker_ptr_;
  std::shared_ptr<FeatureManager> feature_manager_ptr_;

  // cloud_pose以对齐到gnss_pose坐标系上，即UTM坐标系
  // bool Update(const CloudData &cloud_data, Eigen::Isometry3f &cloud_pose,
  //            PoseData &gnss_pose_data, PoseData &preint_pose_data);
  // bool SetInitPose(const Eigen::Isometry3f &init_pose);
  // KeyFrame GetNewKF();
  // PoseData GetNewKeyGnss();
  // PoseData GetNewKeyPreint();

  //#if DebugCloud
  //  const CloudData::CLOUD_PTR GetLocalMap() const { return local_map_ptr_; }
  //  const CloudData::CLOUD_PTR GetFilteredLocalMap() const {
  //    // return filtered_local_map_ptr_;
  //    // return line_extract_ptr_->cloud_2d.CloudPtr();
  //    return registration_ptr_->cloud_dbg;
  //  }
  //#endif

  // public:
  //  static long unsigned int mnKFId;
  //  bool has_new_key_frame_ = false;
  //  bool use_fusion_velocity_ = false; //是否使用IMU/轮速计融合速度
  //
  // private:
  // bool InitWithConfig();
  // bool InitGraphOptimizer(const YAML::Node &config_node);
  // bool InitLineExtract(const YAML::Node &config_node);
  // bool
  // InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr,
  //                 const YAML::Node &config_node);
  // bool InitDataPath(const YAML::Node &config_node);
  // bool InitFilter(std::string filter_user,
  //                std::shared_ptr<CloudFilterInterface> &filter_ptr,
  //                const YAML::Node &config_node);
  // bool UpdateWithNewFrame(const Frame &new_key_frame);
  // bool UpdateInitMap2WorldTF(Eigen::Vector3f odom_trans,
  //                           Eigen::Vector3f gnss_trans);
  // bool Scan2MapMatching(const CloudData::CLOUD_PTR filtered_cloud_ptr);
  // bool PoseFusion();
  // bool UpdateLocalMap();
  // bool SetLocalMapFilterInput();
  // bool UpdateFiteredLocalMap();
  // bool CheckTurningState();
  // bool AddNewKeyFrame(const Frame &new_key_frame);
  // bool AddGraphEdges();
  // bool AddKeyframe2Graph();

  // private:
  // std::string data_path_ = "";
  // std::string key_frames_path_ = "";
  // std::string graph_optimizer_type_ = "";
  // KeyFrame CurrentKF_;
  // PoseData current_key_gnss_;
  // PoseData current_key_preint_;

  // std::shared_ptr<CloudFilterInterface>
  //    frame_filter_ptr_; //点云帧滤波方法指针,多态实现
  // std::shared_ptr<CloudFilterInterface>
  //    local_map_filter_ptr_; //局部地图滤波方法指针,多态实现
  // std::shared_ptr<RegistrationInterface>
  //    registration_ptr_; //点云配准方法的智能指针,多态实现
  // std::shared_ptr<CloudFilterThread> filter_thread_ptr_;
  //
  // std::shared_ptr<InterfaceGraphOptimizer> global_graph_optimizer_ptr_;

  // std::shared_ptr<LineFeatureExtractionInterface> line_extract_ptr_;

  // class GraphOptimizerConfig {
  // public:
  //  GraphOptimizerConfig() {
  //    delta_roll_pitch_noise.resize(2);
  //    delta_height_noise.resize(1);
  //    world_height_noise.resize(1);
  //  }
  //
  // public:
  //  bool use_preint_restrain = false;
  //  bool use_height_restrain = false;
  //  bool use_drp_restrain = false;
  //  bool use_dh_restrain = false;
  //  bool use_analytic_jacobian = false;
  //
  //  Eigen::Vector2f height_change_range;
  //  Eigen::VectorXf delta_roll_pitch_noise;
  //  Eigen::VectorXf delta_height_noise;
  //  Eigen::VectorXf world_height_noise;
  //};
  // GraphOptimizerConfig graph_optimizer_config_;
  // unsigned int optimize_window_size_ = 2;
  //
  // unsigned int local_map_size_ = 0;
  // std::deque<unsigned int> local_map_points_number_; //局部地图
  //
  // CloudData::CLOUD_PTR local_map_ptr_; //局部地图的点云指针
  // CloudData::CLOUD_PTR filtered_local_map_ptr_;
  // bool local_map_valid = true;
  //
  // bool turning_indicator_ = false;
  // Frame current_frame_;
  // FrameDeque keyframes_opt_;
  //
  // Eigen::Isometry3f init_pose_ = Eigen::Isometry3f::Identity(); //初始化位姿
  // Eigen::Isometry3f last_pose_ = Eigen::Isometry3f::Identity();
  // Eigen::Isometry3f predict_pose_ = Eigen::Isometry3f::Identity();
  // Eigen::Isometry3f IncRTFromLastFrame_ = Eigen::Isometry3f::Identity();
  // Eigen::Isometry3f odom_init_pose_ = Eigen::Isometry3f::Identity();
  // Eigen::Isometry3f last_key_frame_pose_ = Eigen::Isometry3f::Identity();

  // float turning_check_angle_ = 1;  // degree
  // float key_frame_distance_ = 2.0; // m
  // float key_frame_angle_ = 5.0;    // degree
  // float key_frame_distance_normal_ = 2.0;
  // float key_frame_angle_normal_ = 5.0;
  // size_t local_frame_num_straight_ = 20; //局部地图的尺寸
  // size_t local_frame_num_turning_ = 60;  //局部地图的尺寸
  //
  // std::string work_space_path_ = "";
};
} // namespace vslam

#endif
