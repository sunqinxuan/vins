/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 10:41
#
# Filename: dataflow.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_DATAFLOW_HPP_
#define VSLAM_DATAFLOW_HPP_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <set>
#include <stdlib.h>
#include <thread>
#include <vector>
//#include <yaml-cpp/yaml.h>

#include "tools/geometry.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {
class DataFlow {
public:
  DataFlow(ros::NodeHandle &nh, std::string work_space_path);
  ~DataFlow() { thread_.join(); }

  // bool Run();

  void start();
  bool getImageData(double &time, cv::Mat &img0, cv::Mat &img1);
  // bool getPreintIMUData(
  //    const double time,
  //    std::pair<double,
  //              std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>>
  //        &imu_interval);
  bool
  getIMUInterval(const double t0, const double t1,
                 std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>
                     &imu_interval);
  std::deque<IMUMeasureTime> getIMUBuf();

  void pubTrackImage(const double t, const cv::Mat &imgTrack);
  void pubOdometry(const double t, const NavState &nav_state,
                   const bool is_initialized);

private:
  void process();

  bool hasNewData();
  bool handleData();

  void cam0_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void cam1_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
  Eigen::Matrix<double, 6, 1>
  getIMUFromMsg(const sensor_msgs::ImuConstPtr &imu_msg);

  // void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
  //                    Eigen::Vector3d angular_velocity);
  //	Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta);

private:
  std::mutex buff_mutex_;
  std::thread thread_;

  ros::Subscriber sub_cam0_;
  ros::Subscriber sub_cam1_;
  ros::Subscriber sub_imu_;

  std::deque<sensor_msgs::ImageConstPtr> img0_msg_buf_;
  std::deque<sensor_msgs::ImageConstPtr> img1_msg_buf_;
  std::deque<sensor_msgs::ImuConstPtr> imu_msg_buf_;

  double prev_time_ = -1, cur_time_ = -1;

  std::deque<std::pair<double, cv::Mat>> img0_buf_;
  std::deque<std::pair<double, cv::Mat>> img1_buf_;
  std::deque<IMUMeasureTime> imu_buf_;
  // std::deque<std::pair<
  //    double, std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>>>
  //    preint_imu_buf_;

  // std::ofstream map_init_ofs_;
  // bool load_map_from_file_ = false;

  std::string vins_result_path_;

	ros::Publisher pub_image_track_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_path_;
};
} // namespace vslam

#endif
