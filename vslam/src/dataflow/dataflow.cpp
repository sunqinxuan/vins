/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 10:41
#
# Filename: dataflow.cpp
#
# Description:
#
************************************************/

#include "dataflow/dataflow.hpp"

namespace vslam {
DataFlow::DataFlow(ros::NodeHandle &nh, std::string work_space_path) {
  // std::string data_path = work_space_path + "/data/map";
  // if (load_map_from_file_) {
  //  YAML::Node map_config_node = YAML::LoadFile(data_path + "/map.yaml");
  //} else {
  //  if (!FileManager::CreateDirectory(data_path))
  //    std::cerr << "cannot create directory: " << data_path << std::endl;
  //  if (!FileManager::CreateFile(map_init_ofs_, data_path + "/map.yaml"))
  //    std::cerr << "cannot create file: " << data_path + "/map.yaml"
  //              << std::endl;
  //}

  std::string config_file_path = work_space_path + "/config/dataflow.yaml";
  cv::FileStorage config(config_file_path, cv::FileStorage::READ);
  std::string cam0_topic, cam1_topic, imu_topic;
  config["cam0_topic"] >> cam0_topic;
  config["cam1_topic"] >> cam1_topic;
  config["imu0_topic"] >> imu_topic;
  config.release();

  std::cout << "cam0_topic: " << cam0_topic << std::endl;
  std::cout << "cam1_topic: " << cam1_topic << std::endl;
  std::cout << "imu_topic: " << imu_topic << std::endl;

  // YAML::Node config_node = YAML::LoadFile(config_file_path);
  // std::string cam0_topic =
  //    config_node["topics"]["cam0_topic"].as<std::string>();
  // std::string cam1_topic =
  //    config_node["topics"]["cam1_topic"].as<std::string>();
  // std::string imu_topic =
  // config_node["topics"]["imu0_topic"].as<std::string>();

  sub_cam0_ = nh.subscribe(cam0_topic, 100, &DataFlow::cam0_callback, this);
  sub_cam1_ = nh.subscribe(cam1_topic, 100, &DataFlow::cam1_callback, this);
  sub_imu_ = nh.subscribe(imu_topic, 2000, &DataFlow::imu_callback, this,
                          ros::TransportHints().tcpNoDelay());
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
  if (img0_msg_buf_.empty() || img1_msg_buf_.empty() || imu_msg_buf_.empty())
    return false;
  else
    return true;
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

} // namespace vslam
