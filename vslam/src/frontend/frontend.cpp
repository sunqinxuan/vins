/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-07-03 09:48
#
# Filename:		frontend.cpp
#
# Description:
#
************************************************/

#include "frontend/frontend.hpp"
//#include "models/graph_optimizer/ceres/ceres_graph_optimizer.hpp"
//
//#include <chrono>
//#include <fstream>
//#include <pcl/common/transforms.h>
//#include <pcl/io/pcd_io.h>
//
//#include "models/cloud_filter/grid_filter.hpp"
//#include "models/cloud_filter/no_filter.hpp"
//#include "models/cloud_filter/voxel_filter.hpp"
//
//#ifdef CUDA_FOUND
//#include "models/cuda/cloud_filter/voxel_filter_gpu.hpp"
//#endif
//
//#include "models/registration/gicp_registration.hpp"
//#include "models/registration/icp_registration.hpp"
//#include "models/registration/icp_svd_registration.h"
//#include "models/registration/icp_withnormal_registration.hpp"
//#include "models/registration/ndt_omp_registration.hpp"
//#include "models/registration/ndt_registration.hpp"
//#include "models/registration/plicp2d_registration.hpp"
//#include "models/registration/plicp_registration.hpp"
//#include "models/registration/sgicp_registration.hpp"
//#include "models/registration/sicp_registration.h"
//#include "tools/convert_matrix.hpp"
//#include "tools/file_manager.hpp"
//#include "tools/tic_toc.hpp"
//
//#include "models/line_feature_extraction/line_feature_extraction_rg.hpp"

namespace vslam {
//全局静态变量
// long unsigned int FrontEnd::mnKFId = 0;

FrontEnd::FrontEnd(ros::NodeHandle &nh, std::string work_space_path) {
  std::string config_file_path = work_space_path + "/config/frontend.yaml";
  // YAML::Node config_node = YAML::LoadFile(config_file_path);

  cv::FileStorage config(config_file_path, cv::FileStorage::READ);

  //	std::string cam0_file;
  //	config["cam0_calib"]>>cam0_file;
  //	DEBUG("cam0_file",cam0_file);
  //	int max_cnt=config["max_cnt"];
  //	DEBUG("max_cnt: ",max_cnt);
  //

  // RIC.push_back(T.block<3, 3>(0, 0));
  // TIC.push_back(T.block<3, 1>(0, 3));
  //

  std::string cam0_file, cam1_file;
  config["cam0_calib"] >> cam0_file;
  config["cam1_calib"] >> cam1_file;
  // std::string cam0_file = config_node["cam0_calib"].as<std::string>();
  // std::string cam1_file = config_node["cam1_calib"].as<std::string>();
  std::string cam0_file_path = work_space_path + "/config" + cam0_file;
  std::string cam1_file_path = work_space_path + "/config" + cam1_file;

  int max_cnt = config["max_cnt"];
  int min_dist = config["min_dist"];
  double F_thres = config["F_threshold"];
  window_size_ = config["window_size"];

  // load extrinsics for camera and imu;
  cv::Mat cv_T;
  config["body_T_cam0"] >> cv_T;
  cv::cv2eigen(cv_T, Tic0_calib_.matrix());
  config["body_T_cam1"] >> cv_T;
  cv::cv2eigen(cv_T, Tic1_calib_.matrix());

  double acc_n = config["acc_n"];
  double gyr_n = config["gyr_n"];
  double acc_w = config["acc_w"];
  double gyr_w = config["gyr_w"];
  double g = config["gravity"];
  gravity.setZero();
  gravity(2) = g;

  double focal_length = config["focal_length"];
  double min_parallax = config["keyframe_parallax"];
  min_parallax = min_parallax / focal_length;

  int num_iter = config["max_num_iterations"];
  double max_time = config["max_solver_time"];
  bool optimizer_debug = config["minimizer_progress_to_stdout"];

  config.release();

  //	gtsam::imuBias::ConstantBias prior_imu_bias;
  // auto params = gtsam::PreintegrationParams::MakeSharedU(gravity);
  // params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
  //    gravity); // TODO
  // params->accelerometerCovariance =
  //    Eigen::Matrix3d::Identity(3, 3) * pow(acc_n, 2);
  // params->gyroscopeCovariance = Eigen::Matrix3d::Identity(3, 3) * pow(gyr_n,
  // 2); params->biasAccCovariance = Eigen::Matrix3d::Identity(3, 3) *
  // pow(acc_w, 2); params->biasOmegaCovariance = Eigen::Matrix3d::Identity(3,
  // 3) * pow(gyr_w, 2); params->integrationCovariance =
  // Eigen::Matrix3d::Identity(3, 3) * 1e-8; params->biasAccOmegaInt =
  // Eigen::Matrix<double, 6, 6>::Identity(6, 6) * 1e-5;

  dt_buf_.resize(window_size_ + 1);
  imu_meas_buf_.resize(window_size_ + 1);

  imu_preint_ptr_ =
      std::make_shared<IMUPreintegration>(acc_n, gyr_n, acc_w, gyr_w, gravity);
  imu_preintegrations_.resize(window_size_ + 1);
  for (auto &imu_preint : imu_preintegrations_) {
    imu_preint = std::make_shared<IMUPreintegration>(acc_n, gyr_n, acc_w, gyr_w,
                                                     gravity);
  }

  feature_manager_ptr_ =
      std::make_shared<FeatureManager>(focal_length, min_parallax);

  clearState();

  dataflow_ptr_ = std::make_shared<DataFlow>(nh, work_space_path);
  dataflow_ptr_->start();

  feature_tracker_ptr_ =
      std::make_shared<FeatureTracker>(max_cnt, min_dist, F_thres);
  feature_tracker_ptr_->readCameraIntrinsics(cam0_file_path, cam1_file_path);

  initial_alignment_ptr_ = std::make_shared<InitialAlignment>();

  graph_optimizer_ptr_ = std::make_shared<FactorGraphOptimizer>(
      num_iter, max_time, optimizer_debug);

  pub_track_image_ =
      nh.advertise<sensor_msgs::Image>("frontend_track_image", 1000);

  thread_track_ = std::thread(&FrontEnd::startTracking, this);
}

void FrontEnd::clearState() {
  std::unique_lock<std::mutex> lock(buff_mutex_);

  while (!feature_buf_.empty())
    feature_buf_.pop();

  track_img_cnt_ = 0;
  frame_cnt_ = 0;
  cur_time_ = 0;
  prev_time_ = -1;
  margin_old_ = false;
  init_first_pose_flag_ = false;
  is_initialized_flag_ = false;
  received_imu_ = false;

  time_stamps_.resize(window_size_ + 1);
  nav_states_.resize(window_size_ + 1);
  imu_bias_.resize(window_size_ + 1);
  // Rs.resize(window_size_ + 1);
  // Ps.resize(window_size_ + 1);
  // Vs.resize(window_size_ + 1);
  // Bas.resize(window_size_ + 1);
  // Bgs.resize(window_size_ + 1);
  for (int i = 0; i < window_size_ + 1; i++) {
    nav_states_[i].clearState();
    imu_bias_[i].setZero();
    dt_buf_.clear();
    imu_meas_buf_.clear();
    // Rs[i].setIdentity();
    // Ps[i].setZero();
    // Vs[i].setZero();
    // Bas[i].setZero();
    // Bgs[i].setZero();
  }

  feature_manager_ptr_->clearState();
  imu_preint_ptr_->reset();
  for (auto &p : imu_preintegrations_) {
    p->reset();
  }
  // imu_preint_ptr_->resetIntegration();
  // for (auto &p : imu_preintegrations_) {
  //  p->resetIntegration();
  //}
}

void FrontEnd::run() {
  std::pair<double, PointsTrack> feature;
  std::vector<IMUMeasureTime> imu_interval;
  while (true) {
    if (feature_buf_.empty()) {
      //			usleep(2 * 1000);
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
      continue;
    } else {
      // std::unique_lock<std::mutex> lock(buff_mutex_);
      buff_mutex_.lock();
      feature = feature_buf_.front();
      feature_buf_.pop();
      buff_mutex_.unlock();

      cur_time_ = feature.first;
      imu_interval.clear();
      if (!dataflow_ptr_->getIMUInterval(prev_time_, cur_time_, imu_interval)) {
        continue;
      }

      // std::cout << "frame_cnt_ = " << frame_cnt_ << std::endl;
      REMIND("frame_cnt_ = ", frame_cnt_);
      INFO("cur_time: ", cur_time_);
      // INFO("prev_time: ", prev_time_);
      // INFO("imu time: ", imu_interval.first);
      // std::cout << "imu_interval.size() = " << imu_interval.size() <<
      // std::endl;
      // for (int i = 0; i < imu_interval.second.size(); i++) {
      // std::cout << std::fixed << "\t" << imu_interval.second[i].first
      //           << std::endl;
      //}

      // process IMU
      if (!init_first_pose_flag_) {
        initIMUPose(imu_interval);
      }
      preintIMU(imu_interval);

      // process image
      processFrame(feature.first, feature.second);

      prev_time_ = cur_time_;

      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
}

void FrontEnd::preintIMU(const std::vector<IMUMeasureTime> &imu_intv) {
  for (size_t i = 0; i < imu_intv.size(); i++) {
    double dt;
    if (i == 0)
      dt = imu_intv[i].first - prev_time_;
    else if (i == imu_intv.size() - 1)
      dt = cur_time_ - imu_intv[i - 1].first;
    else
      dt = imu_intv[i].first - imu_intv[i - 1].first;
    // std::cout << "\t" << std::fixed << imu_intv[i].first << "," << dt
    //          << std::endl;
    IMUMeasure imu = imu_intv[i].second;
    if (!received_imu_) {
      received_imu_ = true;
      imu_meas_0_ = imu;
    }
    // if (imu_preintegrations_[frame_cnt_] == nullptr) {
    //  imu_preintegrations_[frame_cnt_] = std::make_shared<IMUPreintegration>(
    //      imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
    //      imu_bias_[frame_cnt_].head<3>(), imu_bias_[frame_cnt_].tail<3>());
    //}
    if (imu_preintegrations_[frame_cnt_]->empty()) {
      imu_preintegrations_[frame_cnt_]->reset(
          imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
          imu_bias_[frame_cnt_].head<3>(), imu_bias_[frame_cnt_].tail<3>());
    }
    if (frame_cnt_ != 0) {
      imu_preintegrations_[frame_cnt_]->push_back(dt, imu.head<3>(),
                                                  imu.tail<3>());
      imu_preint_ptr_->push_back(dt, imu.head<3>(), imu.tail<3>());

      dt_buf_[frame_cnt_].push_back(dt);
      imu_meas_buf_[frame_cnt_].push_back(imu);

      nav_states_[frame_cnt_].update(dt, imu, imu_meas_0_,
                                     imu_bias_[frame_cnt_], gravity);
      // imu_preint_ptr_->integrateMeasurement(imu.head<3>(), imu.tail<3>(),
      // dt);
      //			imu_preint->integrateMeasurement(imu.head<3>(),imu.tail<3>(),dt);
      // imu_preintegrations_[frame_cnt_]->integrateMeasurement(imu.head<3>(),
      //                                                       imu.tail<3>(),
      //                                                       dt);
      // imu_preint_ptr_->integrateMeasurement(imu.head<3>(), imu.tail<3>(),
      // dt);
    }
    imu_meas_0_ = imu;
  }
  // if (frame_cnt_ != 0)
  //  // nav_states_[frame_cnt_] =
  //  // imu_preint_ptr_->predict(nav_states_[frame_cnt_],
  //  imu_bias_[frame_cnt_]);
  //  // nav_states_[frame_cnt_] = imu_preint->predict(nav_states_[frame_cnt_],
  //  // imu_bias_[frame_cnt_]);
  //  nav_states_[frame_cnt_] = imu_preintegrations_[frame_cnt_]->predict(
  //      nav_states_[frame_cnt_], imu_bias_[frame_cnt_]);
  std::cout << "nav_state: " << std::endl
            << nav_states_[frame_cnt_] << std::endl;
}

void FrontEnd::processFrame(const double &time, const PointsTrack &points) {
  printf("new image coming ------------------------------------------\n");
  printf("Adding feature points %lu\n", points.size());

  if (feature_manager_ptr_->addFeatureCheckParallax(frame_cnt_, points, 0.0)) {
    margin_old_ = true;
  } else {
    margin_old_ = false;
  }
  INFO("margin flag: ", !margin_old_);
  INFO("num of features: ", feature_manager_ptr_->getFeatureCount());
  time_stamps_[frame_cnt_] = time;

  ImageFrame imgframe(points, time);
  imgframe.imu_preint_ptr_ = imu_preint_ptr_->clone();
  image_frames_.insert(std::make_pair(time, imgframe));
  imu_preint_ptr_->reset(imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
                         imu_bias_[frame_cnt_].head<3>(),
                         imu_bias_[frame_cnt_].tail<3>());

  if (!is_initialized_flag_) {
    WARNING(nav_states_[frame_cnt_]);
    REMIND(Tic0_calib_.matrix());
    feature_manager_ptr_->initFramePoseByPnP(frame_cnt_, nav_states_,
                                             Tic0_calib_);
    feature_manager_ptr_->triangulate(nav_states_, Tic0_calib_, Tic1_calib_);
    if (frame_cnt_ == window_size_) {
      std::map<double, ImageFrame>::iterator it_frame;
      int i = 0;
      for (it_frame = image_frames_.begin(); it_frame != image_frames_.end();
           ++it_frame) {
        it_frame->second.R = nav_states_[i].rotation();
        it_frame->second.T = nav_states_[i].position();
        i++;
      }
      initial_alignment_ptr_->solveGyroscopeBias(image_frames_, imu_bias_);
      for (int i = 0; i < window_size_; i++) {
        // imu_preintegrations_[i]->biasCorrectedDelta(imu_bias_[i]);
        imu_preintegrations_[i]->repropagate(Eigen::Vector3d::Zero(),
                                             imu_bias_[i].tail<3>());
      }

      optimize();

      // optimization();
      // updateLatestStates();
      // solver_flag = NON_LINEAR;
      // slideWindow();
      // ROS_INFO("Initialization finish!");
    }

    if (frame_cnt_ < window_size_) {
      frame_cnt_++;
      int prev = frame_cnt_ - 1;
      nav_states_[frame_cnt_] = nav_states_[prev];
      imu_bias_[frame_cnt_] = imu_bias_[prev];
    }
  } else // if(is_initialized_flag_)
  {
  }
}

void FrontEnd::optimize() {
  for (int i = 0; i < frame_cnt_ + 1; i++) {
    graph_optimizer_ptr_->addVertexNavState(i, nav_states_[i]);
  }
  graph_optimizer_ptr_->addVertexCameraEx(0, Tic0_calib_, true);
  graph_optimizer_ptr_->addVertexCameraEx(1, Tic1_calib_, true);

  Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
  for (int i = 0; i < feature_manager_ptr_->getFeatureCount(); i++) {
    graph_optimizer_ptr_->addVertexFeature(i, depth_vec(i));
  }

  // TODO
  // graph_optimizer_ptr_->addFactorMargin();

  for (int i = 0; i < frame_cnt_; i++) {
    int j = i + 1;
    graph_optimizer_ptr_->addFactorIMU(i, j, imu_preintegrations_[i]);
  }

  int f_m_cnt = 0;
  int feature_index = -1;
  for (auto &it_per_id : feature_manager_ptr_->feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;

    ++feature_index;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point;
      Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
      if (imu_i != imu_j) {
        graph_optimizer_ptr_->addFactorProject21(
            imu_i, imu_j, feature_index, pts_i, pts_j,
            it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
        graph_optimizer_ptr_->addFactorProject22(
            imu_i, imu_j, feature_index, pts_i, pts_j_right,
            it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
      } else {
        graph_optimizer_ptr_->addFactorProject12(
            feature_index, pts_i, pts_j_right,
            it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
      }
      f_m_cnt++;
    }
  }
  DEBUG("visual measurement count: ", f_m_cnt);

  if (!graph_optimizer_ptr_->optimize()) {
    WARNING("optimization failure!");
  }

  // TODO
  // line1158: Estimator::double2vector();
  // graph_optimizer_ptr_->getOptimizedStates();

  if (frame_cnt_ < window_size_)
    return;

  // TODO
  // line1165:
  if (margin_old_) {
  } else {
  }
}

void FrontEnd::startTracking() {
  cv::Mat img0, img1;
  double t;
  while (true) {
    if (dataflow_ptr_->getImageData(t, img0, img1)) {
      // INFO("getImageData", t);
      // cv::imshow("img0", img0);
      // cv::waitKey(1);

      trackImage(t, img0, img1);

    } else {
      // usleep(2 * 1000);
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
}

void FrontEnd::trackImage(double t, const cv::Mat &img0, const cv::Mat &img1) {
  track_img_cnt_++;
  PointsTrack frame_features;
  TicToc featureTrackerTime;
  feature_tracker_ptr_->trackImage(t, img0, img1, frame_features);

  // show image track results;
  cv::Mat imgTrack = feature_tracker_ptr_->getTrackImage();
  pubTrackImage(imgTrack, t);

  if (track_img_cnt_ % 2 == 0) {
    std::unique_lock<std::mutex> lock(buff_mutex_);
    feature_buf_.push(std::make_pair(t, frame_features));
  }
}

void FrontEnd::pubTrackImage(const cv::Mat &imgTrack, const double t) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg =
      cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
  pub_track_image_.publish(imgTrackMsg);
}

void FrontEnd::initIMUPose(std::vector<IMUMeasureTime> &imu_intv) {
  std::cout << "init imu pose" << std::endl;
  init_first_pose_flag_ = true;
  Eigen::Vector3d aveAcc(0, 0, 0);
  int n = (int)imu_intv.size();
  for (size_t i = 0; i < imu_intv.size(); i++) {
    aveAcc += imu_intv[i].second.head(3);
  }
  aveAcc /= n;
  std::cout << "average acc: " << aveAcc.transpose() << std::endl;
  Eigen::Matrix3d R0 = Converter::g2R(aveAcc);
  double yaw = Converter::R2ypr(R0).x();
  R0 = Converter::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  // Rs[0] = R0;
  nav_states_[0].setRotation(R0);
  std::cout << "init nav_state = " << std::endl << nav_states_[0] << std::endl;
}

} // namespace vslam
