/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-11 09:22
#
# Filename:		dataflow.cpp
#
# Description:
#
************************************************/

#include "dataflow/dataflow.hpp"

namespace vslam {

DataFlow::DataFlow(ros::NodeHandle &nh, std::string config_file_path) {

  std::string config_file = config_file_path + "config.yaml";
  cv::FileStorage config(config_file, cv::FileStorage::READ);
  std::string cam0_topic, cam1_topic, imu_topic, out_path;
  config["cam0_topic"] >> cam0_topic;
  config["cam1_topic"] >> cam1_topic;
  config["imu0_topic"] >> imu_topic;
  config["output_path"] >> out_path;
  config["use_imu"] >> use_imu_;
  window_size_ = config["window_size"];
  config.release();

  std::cout << "cam0_topic: " << cam0_topic << std::endl;
  std::cout << "cam1_topic: " << cam1_topic << std::endl;
  std::cout << "imu_topic: " << imu_topic << std::endl;
  vins_result_path_ = config_file_path + out_path + "vio.csv";
  std::cout << "result path: " << vins_result_path_ << std::endl;
  std::ofstream fout(vins_result_path_, std::ios::out);
  fout.close();

  sub_cam0_ = nh.subscribe(cam0_topic, 100, &DataFlow::cam0_callback, this);
  sub_cam1_ = nh.subscribe(cam1_topic, 100, &DataFlow::cam1_callback, this);
  if (use_imu_)
    sub_imu_ = nh.subscribe(imu_topic, 2000, &DataFlow::imu_callback, this,
                            ros::TransportHints().tcpNoDelay());

  pub_track_image_ = nh.advertise<sensor_msgs::Image>("/vio/track_image", 1000);
  pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/vio/odometry", 1000);
  pub_path_ = nh.advertise<nav_msgs::Path>("/vio/path", 1000);
  pub_key_poses_ =
      nh.advertise<visualization_msgs::Marker>("/vio/key_poses", 1000);
  pub_camera_pose_ = nh.advertise<nav_msgs::Odometry>("/vio/camera_pose", 1000);
  pub_camera_pose_visual_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/vio/camera_pose_visual", 1000);
  pub_point_cloud_ =
      nh.advertise<sensor_msgs::PointCloud>("/vio/point_cloud", 1000);
  pub_margin_cloud_ =
      nh.advertise<sensor_msgs::PointCloud>("/vio/margin_cloud", 1000);
  pub_keyframe_pose_ =
      nh.advertise<nav_msgs::Odometry>("/vio/keyframe_pose", 1000);
  pub_keyframe_point_ =
      nh.advertise<sensor_msgs::PointCloud>("/vio/keyframe_point", 1000);
  pub_extrinsic_ = nh.advertise<nav_msgs::Odometry>("/vio/extrinsic", 1000);

  cameraposevisual = CameraPoseVisualization(1, 0, 0, 1);
  cameraposevisual.setScale(0.1);
  cameraposevisual.setLineWidth(0.01);
}

void DataFlow::cam0_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  buff_mutex_.lock();
  //	std::cout<<"receive img0"<<std::endl;
  img0_msg_buf_.push_back(img_msg);
  buff_mutex_.unlock();
}

void DataFlow::cam1_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  buff_mutex_.lock();
  //	std::cout<<"receive img1"<<std::endl;
  img1_msg_buf_.push_back(img_msg);
  buff_mutex_.unlock();
}

void DataFlow::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  buff_mutex_.lock();
  //	std::cout<<"receive imu"<<std::endl;
  imu_msg_buf_.push_back(imu_msg);
  buff_mutex_.unlock();
}

cv::Mat DataFlow::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

Eigen::Matrix<double, 6, 1>
DataFlow::getIMUFromMsg(const sensor_msgs::ImuConstPtr &imu_msg) {
  Eigen::Matrix<double, 6, 1> imu;
  imu(0) = imu_msg->linear_acceleration.x;
  imu(1) = imu_msg->linear_acceleration.y;
  imu(2) = imu_msg->linear_acceleration.z;
  imu(3) = imu_msg->angular_velocity.x;
  imu(4) = imu_msg->angular_velocity.y;
  imu(5) = imu_msg->angular_velocity.z;
  return imu;
}

// bool DataFlow::Run() {
//  //	std::cout<<"DataFlow::Run()"<<std::endl;
//  if (!handleData())
//    return false;
//
//  std::cout << "current img buf:" << std::endl;
//  for (int i = 0; i < img0_buf_.size(); i++) {
//    std::cout << std::fixed << "\t" << img0_buf_.at(i).first << std::endl;
//  }
//  std::cout << "current imu buf:" << std::endl;
//  for (int i = 0; i < imu_buf_.size(); i++) {
//    std::cout << std::fixed << "\t" << imu_buf_.at(i).first << std::endl;
//  }
//  std::cout << "current imu preint buf:" << std::endl;
//  for (int i = 0; i < preint_imu_buf_.size(); i++) {
//    std::cout << std::fixed << "\t" << preint_imu_buf_.at(i).first <<
//    std::endl; for (int j = 0; j < preint_imu_buf_.at(i).second.size(); j++) {
//      std::cout << std::fixed << "\t\t"
//                << preint_imu_buf_.at(i).second.at(j).first << std::endl;
//    }
//  }
//
//  return true;
//}

void DataFlow::start() {
  // std::cout << "start begin" << std::endl;
  thread_ = std::thread(&DataFlow::process, this);
  // std::cout << "start end" << std::endl;
}

bool DataFlow::getImageData(double &time, cv::Mat &img0, cv::Mat &img1) {
  if (img0_buf_.empty() || img1_buf_.empty())
    return false;
  std::unique_lock<std::mutex> lock(buff_mutex_);
  time = img0_buf_.front().first;
  img0 = img0_buf_.front().second;
  img1 = img1_buf_.front().second;
  img0_buf_.pop_front();
  img1_buf_.pop_front();
  return true;
}

// bool DataFlow::getPreintIMUData(
//    const double time,
//    std::pair<double,
//              std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>>
//        &imu_interval) {
//  if (preint_imu_buf_.empty())
//    return false;
//  if (time < preint_imu_buf_.front().first - 0.01)
//    return false;
//
//  std::unique_lock<std::mutex> lock(buff_mutex_);
//  for (size_t i = 0; i < preint_imu_buf_.size(); i++) {
//    if (fabs(time - preint_imu_buf_.at(i).first) < 0.01) {
//      imu_interval = preint_imu_buf_.at(i);
//      for (size_t j = 0; j <= i; j++)
//        preint_imu_buf_.pop_front();
//      return true;
//    }
//  }
//  return false;
//}

void DataFlow::process() {
  // std::cout << "process begin" << std::endl;
  while (true) {
    ros::spinOnce();
    if (hasNewData()) {
      handleData();
      // std::cout << "====================process==================="
      //          << std::endl;
      // std::cout << "current img buf:" << std::endl;
      // for (int i = 0; i < img0_buf_.size(); i++) {
      //  std::cout << std::fixed << "\t" << img0_buf_.at(i).first << std::endl;
      //}
      // std::cout << "current imu buf:" << std::endl;
      // for (int i = 0; i < imu_buf_.size(); i++) {
      //  std::cout << std::fixed << "\t" << imu_buf_.at(i).first << std::endl;
      //}
      // std::cout << "current imu preint buf:" << std::endl;
      // for (int i = 0; i < preint_imu_buf_.size(); i++) {
      //  std::cout << std::fixed << "\t" << preint_imu_buf_.at(i).first
      //            << std::endl;
      //  for (int j = 0; j < preint_imu_buf_.at(i).second.size(); j++) {
      //    std::cout << std::fixed << "\t\t"
      //              << preint_imu_buf_.at(i).second.at(j).first << std::endl;
      //  }
      //}
    } else {
      // usleep(2 * 1000);
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
}

bool DataFlow::hasNewData() {
  if (use_imu_) {
    if (img0_msg_buf_.empty() || img1_msg_buf_.empty() || imu_msg_buf_.empty())
      return false;
    else
      return true;
  } else {
    if (img0_msg_buf_.empty() || img1_msg_buf_.empty())
      return false;
    else
      return true;
  }
}

bool DataFlow::handleData() {
  // buff_mutex_.lock();
  std::unique_lock<std::mutex> lock(buff_mutex_);
  double time0 = img0_msg_buf_.front()->header.stamp.toSec();
  double time1 = img1_msg_buf_.front()->header.stamp.toSec();
  // std::cout << std::fixed << "img0 time=" << time0 << ", img1 time=" << time1
  //          << std::endl;
  // 0.003s sync tolerance
  if (time0 < time1 - 0.003) {
    img0_msg_buf_.pop_front();
    printf("throw img0\n");
    // buff_mutex_.unlock();
    return false;
  }
  if (time0 > time1 + 0.003) {
    img1_msg_buf_.pop_front();
    printf("throw img1\n");
    // buff_mutex_.unlock();
    return false;
  }

  cur_time_ = time0;
  cv::Mat image0 = getImageFromMsg(img0_msg_buf_.front());
  img0_buf_.push_back(std::make_pair(cur_time_, image0));
  img0_msg_buf_.pop_front();
  cv::Mat image1 = getImageFromMsg(img1_msg_buf_.front());
  img1_buf_.push_back(std::make_pair(cur_time_, image1));
  img1_msg_buf_.pop_front();
  // std::cout << std::fixed << "cur_time=" << cur_time_ << std::endl;
  if (prev_time_ < 0) {
    // first frame;
    prev_time_ = cur_time_;
    // buff_mutex_.unlock();
    return false;
  }
  // std::cout << std::fixed << "prev_time=" << prev_time_ << std::endl;

  if (use_imu_) {
    double imu_front_time = imu_msg_buf_.front()->header.stamp.toSec();
    double imu_back_time = imu_msg_buf_.back()->header.stamp.toSec();
    // std::cout << std::fixed << "imu front=" << imu_front_time
    //          << ", back=" << imu_back_time << std::endl;

    if (imu_back_time < cur_time_ || imu_front_time > cur_time_) {
      prev_time_ = cur_time_;
      // buff_mutex_.unlock();
      return false;
    }

    while (imu_msg_buf_.size() > 0) {
      Eigen::Matrix<double, 6, 1> imu = getIMUFromMsg(imu_msg_buf_.front());
      double time = imu_msg_buf_.front()->header.stamp.toSec();
      imu_buf_.push_back(std::make_pair(time, imu));
      imu_msg_buf_.pop_front();
    }
  }

  // std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>> imu_interval;
  // getIMUInterval(prev_time_, cur_time_, imu_interval);
  // preint_imu_buf_.push_back(std::make_pair(cur_time_, imu_interval));

  prev_time_ = cur_time_;
  // buff_mutex_.unlock();
  return true;
}

bool DataFlow::getIMUInterval(
    const double t0, const double t1,
    std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>> &imu_interval) {
  // std::cout << "==================getIMUInterval====================="
  //<< std::endl;
  if (imu_buf_.empty()) {
    printf("no imu data in the buffer!\n");
    return false;
  }
  printf("get imu from %f %f\n", t0, t1);
  double imu_front_time = imu_buf_.front().first;
  double imu_back_time = imu_buf_.back().first;
  printf("imu fornt time %f   imu end time %f\n", imu_front_time,
         imu_back_time);
  // for (int i = 0; i < imu_buf_.size(); i++) {
  //  std::cout << std::fixed << "\t" << imu_buf_.at(i).first << std::endl;
  //}
  imu_interval.clear();
  if (t1 <= imu_back_time) {
    while (imu_buf_.front().first <= t0) {
      // std::cout << std::fixed << "1 pop " << imu_buf_.front().first
      //          << std::endl;
      imu_buf_.pop_front();
    }
    while (imu_buf_.front().first < t1) {
      // std::cout << std::fixed << "1 push " << imu_buf_.front().first
      //          << std::endl;
      imu_interval.push_back(imu_buf_.front());
      imu_buf_.pop_front();
    }
    // std::cout << std::fixed << "2 push " << imu_buf_.front().first <<
    // std::endl;
    imu_interval.push_back(imu_buf_.front());
    imu_buf_.pop_front();
  } else {
    printf("waiting for imu ...\n");
    return false;
  }
  // std::cout << "==================getIMUInterval end================="
  //          << std::endl;
  return true;
}

std::deque<IMUMeasureTime> DataFlow::getIMUBuf() {
  std::unique_lock<std::mutex> lock(buff_mutex_);
  std::deque<IMUMeasureTime> buf = imu_buf_;
  return buf;
}

// void DataFlow::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
//                              Eigen::Vector3d angular_velocity) {
//  double dt = t - latest_time;
//  latest_time = t;
//  Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
//  Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) -
//  latest_Bg; latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
//  Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
//  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
//  latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
//  latest_V = latest_V + dt * un_acc;
//  latest_acc_0 = linear_acceleration;
//  latest_gyr_0 = angular_velocity;
//}
//
// Eigen::Quaterniond DataFlow::deltaQ(const Eigen::Vector3d &theta) {
//  Eigen::Quaterniond dq;
//  Eigen::Vector3d half_theta = theta;
//  half_theta /= 2.0;
//  dq.w() = 1.0;
//  dq.x() = half_theta.x();
//  dq.y() = half_theta.y();
//  dq.z() = half_theta.z();
//  return dq;
//}

void DataFlow::pubTrackImage(const double t, const cv::Mat &imgTrack) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg =
      cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
  pub_track_image_.publish(imgTrackMsg);
}

void DataFlow::pubOdometry(const double t, const NavState &nav_state,
                           const bool is_initialized) {
  if (!is_initialized)
    return;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.child_frame_id = "world";

  Eigen::Quaterniond tmp_Q;
  tmp_Q = Eigen::Quaterniond(nav_state.rotation());

  odometry.pose.pose.position.x = nav_state.position().x();
  odometry.pose.pose.position.y = nav_state.position().y();
  odometry.pose.pose.position.z = nav_state.position().z();
  odometry.pose.pose.orientation.x = tmp_Q.x();
  odometry.pose.pose.orientation.y = tmp_Q.y();
  odometry.pose.pose.orientation.z = tmp_Q.z();
  odometry.pose.pose.orientation.w = tmp_Q.w();
  odometry.twist.twist.linear.x = nav_state.velocity().x();
  odometry.twist.twist.linear.y = nav_state.velocity().y();
  odometry.twist.twist.linear.z = nav_state.velocity().z();
  pub_odometry_.publish(odometry);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose = odometry.pose.pose;
  vio_path_.header = header;
  vio_path_.header.frame_id = "world";
  vio_path_.poses.push_back(pose_stamped);
  pub_path_.publish(vio_path_);

  // write result to file
  std::ofstream foutC(vins_result_path_, std::ios::app);
  foutC.setf(std::ios::fixed, std::ios::floatfield);
  foutC.precision(0);
  foutC << header.stamp.toSec() * 1e9 << ",";
  foutC.precision(5);
  foutC << nav_state.position().x() << "," << nav_state.position().y() << ","
        << nav_state.position().z() << "," << tmp_Q.w() << "," << tmp_Q.x()
        << "," << tmp_Q.y() << "," << tmp_Q.z() << ","
        << nav_state.velocity().x() << "," << nav_state.velocity().y() << ","
        << nav_state.velocity().z() << "," << std::endl;
  foutC.close();
}

void DataFlow::pubKeyPoses(const double t,
                           const std::vector<NavState> &nav_states) {
  if (nav_states.size() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);

  visualization_msgs::Marker key_poses;
  key_poses.header = header;
  key_poses.header.frame_id = "world";
  key_poses.ns = "key_poses";
  key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
  key_poses.action = visualization_msgs::Marker::ADD;
  key_poses.pose.orientation.w = 1.0;
  key_poses.lifetime = ros::Duration();

  // static int key_poses_id = 0;
  key_poses.id = 0; // key_poses_id++;
  key_poses.scale.x = 0.05;
  key_poses.scale.y = 0.05;
  key_poses.scale.z = 0.05;
  key_poses.color.r = 1.0;
  key_poses.color.a = 1.0;

  for (size_t i = 0; i < nav_states.size(); i++) {
    geometry_msgs::Point pose_marker;
    Eigen::Vector3d correct_pose;
    correct_pose = nav_states[i].position();
    pose_marker.x = correct_pose.x();
    pose_marker.y = correct_pose.y();
    pose_marker.z = correct_pose.z();
    key_poses.points.push_back(pose_marker);
  }
  pub_key_poses_.publish(key_poses);
}

void DataFlow::pubCameraPose(const double t, const NavState &nav_state,
                             const Eigen::Isometry3d &Tic0,
                             const Eigen::Isometry3d &Tic1,
                             const bool is_initialized) {
  if (!is_initialized)
    return;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);

  // Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
  // Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);
  Eigen::Vector3d P =
      nav_state.position() + nav_state.rotation() * Tic0.translation();
  Eigen::Quaterniond R =
      Eigen::Quaterniond(nav_state.rotation() * Tic0.linear());

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = P.x();
  odometry.pose.pose.position.y = P.y();
  odometry.pose.pose.position.z = P.z();
  odometry.pose.pose.orientation.x = R.x();
  odometry.pose.pose.orientation.y = R.y();
  odometry.pose.pose.orientation.z = R.z();
  odometry.pose.pose.orientation.w = R.w();

  pub_camera_pose_.publish(odometry);

  cameraposevisual.reset();
  cameraposevisual.add_pose(P, R);

  // cam_right
  // Eigen::Vector3d Pr = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
  // Eigen::Quaterniond Rr = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
  Eigen::Vector3d Pr =
      nav_state.position() + nav_state.rotation() * Tic1.translation();
  Eigen::Quaterniond Rr =
      Eigen::Quaterniond(nav_state.rotation() * Tic1.linear());
  cameraposevisual.add_pose(Pr, Rr);

  cameraposevisual.publish_by(pub_camera_pose_visual_, odometry.header);
}

void DataFlow::pubPointCloud(const double t,
                             const std::vector<NavState> &nav_states,
                             const Eigen::Isometry3d &Tic0,
                             const std::list<FeaturePerId> &feature) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);

  sensor_msgs::PointCloud point_cloud, loop_point_cloud;
  point_cloud.header = header;
  loop_point_cloud.header = header;

  for (auto &it_per_id : feature) {
    int used_num;
    used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < window_size_ - 2))
      continue;
    if (it_per_id.start_frame > window_size_ * 3.0 / 4.0 ||
        it_per_id.solve_flag != 1)
      continue;
    int imu_i = it_per_id.start_frame;
    Eigen::Vector3d pts_i =
        it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i = nav_states[imu_i].rotation() *
                                  (Tic0.linear() * pts_i + Tic0.translation()) +
                              nav_states[imu_i].position();
    // Eigen::Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i
    // + estimator.tic[0]) + estimator.Ps[imu_i];

    geometry_msgs::Point32 p;
    p.x = w_pts_i(0);
    p.y = w_pts_i(1);
    p.z = w_pts_i(2);
    point_cloud.points.push_back(p);
  }
  pub_point_cloud_.publish(point_cloud);

  // pub margined potin
  sensor_msgs::PointCloud margin_cloud;
  margin_cloud.header = header;

  // DEBUG("publish margin cloud ==========================");
  // DEBUG("===============================================");
  // using namespace std;
  // cout << "time = " << fixed << t << endl;

  for (auto &it_per_id : feature) {
    int used_num;
    used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < window_size_ - 2))
      continue;
    // if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 ||
    // it_per_id->solve_flag != 1)
    //        continue;

    if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 &&
        it_per_id.solve_flag == 1) {
      int imu_i = it_per_id.start_frame;
      Eigen::Vector3d pts_i =
          it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
      Eigen::Vector3d w_pts_i =
          nav_states[imu_i].rotation() *
              (Tic0.linear() * pts_i + Tic0.translation()) +
          nav_states[imu_i].position();
      // Eigen::Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] *
      // pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
      // cout << "imu_i = " << imu_i << " -----------------------" << endl;
      // cout << "pts_i = " << pts_i.transpose() << endl;
      // cout << "w_pts_i = " << w_pts_i.transpose() << endl;

      geometry_msgs::Point32 p;
      p.x = w_pts_i(0);
      p.y = w_pts_i(1);
      p.z = w_pts_i(2);
      margin_cloud.points.push_back(p);
    }
  }
  pub_margin_cloud_.publish(margin_cloud);
}

void DataFlow::pubKeyframe(const double t,
                           const std::vector<NavState> &nav_states,
                           const Eigen::Isometry3d &Tic0,
                           const std::list<FeaturePerId> &feature,
                           const bool is_initialized,
                           const bool is_margin_old) {
  // pub camera pose, 2D-3D points of keyframe
  // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR &&
  // estimator.marginalization_flag == 0)
  if (is_initialized && is_margin_old) {
    int i = window_size_ - 2;
    // Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Eigen::Vector3d P = nav_states[i].position();
    Eigen::Quaterniond R = Eigen::Quaterniond(nav_states[i].rotation());

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();
    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
    printf("time: %f t: %f %f %f r: %f %f %f %f\n",
           odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(),
           R.y(), R.z());

    pub_keyframe_pose_.publish(odometry);

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.stamp = ros::Time(t);
    point_cloud.header.frame_id = "world";
    for (auto &it_per_id : feature) {
      int frame_size = it_per_id.feature_per_frame.size();
      if (it_per_id.start_frame < window_size_ - 2 &&
          it_per_id.start_frame + frame_size - 1 >= window_size_ - 2 &&
          it_per_id.solve_flag == 1) {

        int imu_i = it_per_id.start_frame;
        Eigen::Vector3d pts_i =
            it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Eigen::Vector3d w_pts_i =
            nav_states[imu_i].rotation() *
                (Tic0.linear() * pts_i + Tic0.translation()) +
            nav_states[imu_i].position();
        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);

        int imu_j = window_size_ - 2 - it_per_id.start_frame;
        sensor_msgs::ChannelFloat32 p_2d;
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
        p_2d.values.push_back(it_per_id.feature_id);
        point_cloud.channels.push_back(p_2d);
      }
    }
    pub_keyframe_point_.publish(point_cloud);
  }
}

void DataFlow::pubTF(const double t, const std::vector<NavState> &nav_states,
                     const Eigen::Isometry3d &Tic0, const bool is_initialized) {
  if (!is_initialized)
    return;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  // body frame
  Eigen::Vector3d correct_t;
  Eigen::Quaterniond correct_q;
  correct_t = nav_states[window_size_].position();
  correct_q = nav_states[window_size_].rotation();

  transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, "world", "body"));

  // camera frame
  transform.setOrigin(tf::Vector3(
      Tic0.translation().x(), Tic0.translation().y(), Tic0.translation().z()));
  q.setW(Eigen::Quaterniond(Tic0.linear()).w());
  q.setX(Eigen::Quaterniond(Tic0.linear()).x());
  q.setY(Eigen::Quaterniond(Tic0.linear()).y());
  q.setZ(Eigen::Quaterniond(Tic0.linear()).z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, "body", "camera"));

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = Tic0.translation().x();
  odometry.pose.pose.position.y = Tic0.translation().y();
  odometry.pose.pose.position.z = Tic0.translation().z();
  Eigen::Quaterniond tmp_q{Tic0.linear()};
  odometry.pose.pose.orientation.x = tmp_q.x();
  odometry.pose.pose.orientation.y = tmp_q.y();
  odometry.pose.pose.orientation.z = tmp_q.z();
  odometry.pose.pose.orientation.w = tmp_q.w();

  pub_extrinsic_.publish(odometry);
}
} // namespace vslam
