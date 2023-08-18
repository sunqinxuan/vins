/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 15:03
#
# Filename:		loopfusion.cpp
#
# Description:
#
************************************************/

#include "loopfusion/loopfusion.hpp"

namespace vslam {

std::string CONFIG_FILE_PATH;
ros::Publisher PUB_MATCH_IMG;
Eigen::Isometry3d Tic0_calib_;

LoopFusion::LoopFusion(ros::NodeHandle &nh, std::string config_file_path) {

  CONFIG_FILE_PATH = config_file_path;
  std::string config_file = config_file_path + "config.yaml";
  INFO("[LoopFusion] config_file: ", config_file);
  cv::FileStorage config(config_file, cv::FileStorage::READ);
  if (!config.isOpened()) {
    ERROR("[LoopFusion] ERROR: wrong path to config file!");
  }

  //	bool debug_image;
  //	config["debug_image"]>>debug_image;

  // std::string cam0_file;
  // config["cam0_calib"] >> cam0_file;
  // std::string cam0_file_path = config_file_path + cam0_file;
  // std::cout << "cam0_file_path: " << cam0_file_path << std::endl;
  int use_imu = config["use_imu"];
  //	std::string output_path;
  //  config["output_path"] >> out_path;
  config.release();

  clearState();

  pose_graph_ptr_ = std::make_shared<PoseGraph>(config_file_path);
  pose_graph_ptr_->registerPub(nh);

  std::string vocabulary_file = config_file_path + "brief_k10L6.bin";
  INFO("vocabulary_file: ", vocabulary_file);
  pose_graph_ptr_->loadVocabulary(vocabulary_file);
  pose_graph_ptr_->setIMUFlag(use_imu);

  dataflow_ptr_ =
      std::make_shared<LoopDataFlow>(nh, config_file_path, pose_graph_ptr_);
  dataflow_ptr_->start();

  thread_track_ = std::thread(&LoopFusion::startTracking, this);

  PUB_MATCH_IMG =
      nh.advertise<sensor_msgs::Image>("/loopfusion/match_image", 1000);
}

void LoopFusion::clearState() {
  std::unique_lock<std::mutex> lock(buff_mutex_);
}

void LoopFusion::run() {

  while (true) {
    // if (flag_)
    //  break;

    // if (feature_buf_.empty()) {
    //  std::chrono::milliseconds dura(2);
    //  std::this_thread::sleep_for(dura);
    //  continue;
    //} else {

    // process image
    // processFrame(feature.first, feature.second);

    // prev_time_ = cur_time_;

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
    //}
  }
}

void LoopFusion::startTracking() {
  cv::Mat img0, img1;
  double t;
  while (true) {
    // if (dataflow_ptr_->getImageData(t, img0, img1)) {
    // INFO("getImageData", t);
    // cv::imshow("img0", img0);
    // cv::waitKey(1);

    // trackImage(t, img0, img1);

    //} else {
    // usleep(2 * 1000);
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
    //}
  }
}

void LoopFusion::trackImage(double t, const cv::Mat &img0,
                            const cv::Mat &img1) {
  // track_img_cnt_++;
  // PointsTrack frame_features;
  // TicToc featureTrackerTime;
  // feature_tracker_ptr_->trackImage(t, img0, img1, frame_features);
  //
  //// show image track results;
  // cv::Mat imgTrack = feature_tracker_ptr_->getTrackImage();
  // dataflow_ptr_->pubTrackImage(t, imgTrack);
  //
  // if (track_img_cnt_ % 2 == 0) {
  //  std::unique_lock<std::mutex> lock(buff_mutex_);
  //  feature_buf_.push(std::make_pair(t, frame_features));
  //}
}

} // namespace vslam
