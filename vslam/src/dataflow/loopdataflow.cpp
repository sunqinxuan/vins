/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 14:00
#
# Filename:		loopdataflow.cpp
#
# Description:
#
************************************************/

#include "dataflow/loopdataflow.hpp"

namespace vslam {

LoopDataFlow::LoopDataFlow(ros::NodeHandle &nh, std::string config_file_path,
                           std::shared_ptr<PoseGraph> pose_graph_ptr)
    : pose_graph_ptr_(pose_graph_ptr) {

  std::string config_file = config_file_path + "config.yaml";
  cv::FileStorage config(config_file, cv::FileStorage::READ);

  std::string cam0_topic;
  config["cam0_topic"] >> cam0_topic;
  config.release();

  sub_image_ =
      nh.subscribe(cam0_topic, 2000, &LoopDataFlow::cam0_callback, this);
  sub_vio_ =
      nh.subscribe("/vio/odometry", 2000, &LoopDataFlow::vio_callback, this);
  sub_extrinsic_ = nh.subscribe("/vio/extrinsic", 2000,
                                &LoopDataFlow::extrinsic_callback, this);
  sub_keyframe_pose_ = nh.subscribe(
      "/vio/keyframe_pose", 2000, &LoopDataFlow::keyframe_pose_callback, this);
  sub_keyframe_point_ =
      nh.subscribe("/vio/keyframe_point", 2000,
                   &LoopDataFlow::keyframe_point_callback, this);
  sub_margin_point_ = nh.subscribe("/vio/margin_cloud", 2000,
                                   &LoopDataFlow::margin_point_callback, this);

  pub_odometry_rect_ =
      nh.advertise<nav_msgs::Odometry>("/loopfusion/odometry_rect", 1000);

  pub_camera_pose_visual_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/loopfusion/camera_pose_visual", 1000);
  pub_point_cloud_ = nh.advertise<sensor_msgs::PointCloud>(
      "/loopfusion/point_cloud_loop_rect", 1000);
  pub_margin_cloud_ = nh.advertise<sensor_msgs::PointCloud>(
      "/loopfusion/margin_cloud_loop_rect", 1000);

  cameraposevisual = CameraPoseVisualization(1, 0, 0, 1);
  cameraposevisual.setScale(0.1);
  cameraposevisual.setLineWidth(0.01);
}

void LoopDataFlow::start() {
  // std::cout << "start begin" << std::endl;
  thread_ = std::thread(&LoopDataFlow::process, this);
  // std::cout << "start end" << std::endl;
}

void LoopDataFlow::process() {
  // std::cout << "process begin" << std::endl;
  while (true) {
    ros::spinOnce();
    if (hasNewData()) {
      handleData();

    } else {
      // usleep(2 * 1000);
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
}

bool LoopDataFlow::hasNewData() {
  // if (use_imu_) {
  //  if (img0_msg_buf_.empty() || img1_msg_buf_.empty() ||
  //  imu_msg_buf_.empty())
  //    return false;
  //  else
  //    return true;
  //} else {
  //  if (img0_msg_buf_.empty() || img1_msg_buf_.empty())
  //    return false;
  //  else
  return true;
  //}
}

bool LoopDataFlow::handleData() {
  // std::unique_lock<std::mutex> lock(buff_mutex_);

  return true;
}

void LoopDataFlow::cam0_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  buff_mutex_.lock();
  img0_msg_buf_.push_back(img_msg);
  DEBUG("[loopfusion] receive img0!");
  buff_mutex_.unlock();
}

void LoopDataFlow::vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  // ROS_INFO("vio_callback!");
  Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x,
                        pose_msg->pose.pose.position.y,
                        pose_msg->pose.pose.position.z);
  Eigen::Quaterniond vio_q;
  vio_q.w() = pose_msg->pose.pose.orientation.w;
  vio_q.x() = pose_msg->pose.pose.orientation.x;
  vio_q.y() = pose_msg->pose.pose.orientation.y;
  vio_q.z() = pose_msg->pose.pose.orientation.z;

  vio_t = pose_graph_ptr_->w_r_vio * vio_t + pose_graph_ptr_->w_t_vio;
  vio_q = pose_graph_ptr_->w_r_vio * vio_q;

  vio_t = pose_graph_ptr_->r_drift * vio_t + pose_graph_ptr_->t_drift;
  vio_q = pose_graph_ptr_->r_drift * vio_q;

  nav_msgs::Odometry odometry;
  odometry.header = pose_msg->header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = vio_t.x();
  odometry.pose.pose.position.y = vio_t.y();
  odometry.pose.pose.position.z = vio_t.z();
  odometry.pose.pose.orientation.x = vio_q.x();
  odometry.pose.pose.orientation.y = vio_q.y();
  odometry.pose.pose.orientation.z = vio_q.z();
  odometry.pose.pose.orientation.w = vio_q.w();
  pub_odometry_rect_.publish(odometry);

  Eigen::Vector3d vio_t_cam;
  Eigen::Quaterniond vio_q_cam;
  // vio_t_cam = vio_t + vio_q * tic;
  // vio_q_cam = vio_q * qic;
  vio_t_cam = vio_t + vio_q * Tic0_calib_.translation();
  vio_q_cam = vio_q * Tic0_calib_.linear();

  cameraposevisual.reset();
  cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
  cameraposevisual.publish_by(pub_camera_pose_visual_, pose_msg->header);
}

void LoopDataFlow::extrinsic_callback(
    const nav_msgs::Odometry::ConstPtr &pose_msg) {
  proc_mutex_.lock();
  Tic0_calib_.translation() = Eigen::Vector3d(pose_msg->pose.pose.position.x,
                                              pose_msg->pose.pose.position.y,
                                              pose_msg->pose.pose.position.z);
  Tic0_calib_.linear() = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                            pose_msg->pose.pose.orientation.x,
                                            pose_msg->pose.pose.orientation.y,
                                            pose_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
  proc_mutex_.unlock();
}

void LoopDataFlow::keyframe_pose_callback(
    const nav_msgs::Odometry::ConstPtr &pose_msg) {
  // ROS_INFO("pose_callback!");
  buff_mutex_.lock();
  pose_msg_buf_.push_back(pose_msg);
  buff_mutex_.unlock();
  /*
  printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n",
  pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                                                     pose_msg->pose.pose.position.z,
                                                     pose_msg->pose.pose.orientation.w,
                                                     pose_msg->pose.pose.orientation.x,
                                                     pose_msg->pose.pose.orientation.y,
                                                     pose_msg->pose.pose.orientation.z);
  */
}

void LoopDataFlow::keyframe_point_callback(
    const sensor_msgs::PointCloudConstPtr &point_msg) {
  // ROS_INFO("point_callback!");
  buff_mutex_.lock();
  point_msg_buf_.push_back(point_msg);
  buff_mutex_.unlock();
  /*
  for (unsigned int i = 0; i < point_msg->points.size(); i++)
  {
      printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i ,
  point_msg->points[i].x, point_msg->points[i].y, point_msg->points[i].z,
                                                   point_msg->channels[i].values[0],
                                                   point_msg->channels[i].values[1]);
  }
  */
  // for visualization
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        pose_graph_ptr_->r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        pose_graph_ptr_->t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_point_cloud_.publish(point_cloud);
}

void LoopDataFlow::margin_point_callback(
    const sensor_msgs::PointCloudConstPtr &point_msg) {
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        pose_graph_ptr_->r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        pose_graph_ptr_->t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_margin_cloud_.publish(point_cloud);
}

} // namespace vslam
