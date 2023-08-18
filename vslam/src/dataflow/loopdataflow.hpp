/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 14:00
#
# Filename:		loopdataflow.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_LOOPDATAFLOW_HPP_
#define VSLAM_LOOPDATAFLOW_HPP_

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "dataflow/camera_visualization.hpp"
#include "loopfusion/pose_graph.hpp"
#include "tools/geometry.hpp"
#include "tools/message_print.hpp"
#include "tools/type_redefine.hpp"

namespace vslam {

// extern Eigen::Isometry3d Tic0_calib_;

class LoopDataFlow {
public:
  LoopDataFlow() = delete;
  LoopDataFlow(ros::NodeHandle &nh, std::string config_file_path,
               std::shared_ptr<PoseGraph> pose_graph_ptr);
  ~LoopDataFlow() { thread_.join(); }

  void start();

  Eigen::Isometry3d getTic0() const { return Tic0_calib_; }

private:
  void process();

  bool hasNewData();
  bool handleData();

  void cam0_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
  void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);

  void keyframe_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
  void
  keyframe_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg);
  void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg);

private:
  std::mutex buff_mutex_;
  std::mutex proc_mutex_;
  std::thread thread_;

  Eigen::Isometry3d Tic0_calib_;

  std::deque<sensor_msgs::ImageConstPtr> img0_msg_buf_;
  std::deque<nav_msgs::Odometry::ConstPtr> pose_msg_buf_;
  std::deque<sensor_msgs::PointCloudConstPtr> point_msg_buf_;

  CameraPoseVisualization cameraposevisual;
  std::shared_ptr<PoseGraph> pose_graph_ptr_;

  ros::Subscriber sub_image_;
  ros::Subscriber sub_vio_;
  ros::Subscriber sub_extrinsic_;
  ros::Subscriber sub_keyframe_pose_;
  ros::Subscriber sub_keyframe_point_;
  ros::Subscriber sub_margin_point_;

  ros::Publisher pub_odometry_rect_;
  ros::Publisher pub_camera_pose_visual_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_margin_cloud_;
};
} // namespace vslam

#endif
