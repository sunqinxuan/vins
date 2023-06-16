/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-15 09:24
#
# Filename: frontend.cpp
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
  //YAML::Node config_node = YAML::LoadFile(config_file_path);

	cv::FileStorage config(config_file_path,cv::FileStorage::READ);

//	std::string cam0_file;
//	config["cam0_calib"]>>cam0_file;
//	DEBUG("cam0_file",cam0_file);
//	int max_cnt=config["max_cnt"];
//	DEBUG("max_cnt: ",max_cnt);
//

        //RIC.push_back(T.block<3, 3>(0, 0));
        //TIC.push_back(T.block<3, 1>(0, 3));
//

	std::string cam0_file, cam1_file;
	config["cam0_calib"]>>cam0_file;
	config["cam1_calib"]>>cam1_file;
  //std::string cam0_file = config_node["cam0_calib"].as<std::string>();
  //std::string cam1_file = config_node["cam1_calib"].as<std::string>();
  std::string cam0_file_path = work_space_path + "/config" + cam0_file;
  std::string cam1_file_path = work_space_path + "/config" + cam1_file;

  int max_cnt = config["max_cnt"];
  int min_dist = config["min_dist"];
  double F_thres = config["F_threshold"];

	// load extrinsics for camera and imu;
	cv::Mat cv_T;
	config["body_T_cam0"] >> cv_T;
	cv::cv2eigen(cv_T, Tic0_calib.matrix());
	config["body_T_cam1"] >> cv_T;
	cv::cv2eigen(cv_T, Tic1_calib.matrix());

	config.release();

  dataflow_ptr_ = std::make_shared<DataFlow>(nh, work_space_path);
  dataflow_ptr_->start();

  feature_tracker_ptr_ =
      std::make_shared<FeatureTracker>(max_cnt, min_dist, F_thres);
  feature_tracker_ptr_->readCameraIntrinsics(cam0_file_path, cam1_file_path);

	feature_manager_ptr_=std::make_shared<FeatureManager>();

  pub_track_image_ =
      nh.advertise<sensor_msgs::Image>("frontend_track_image", 1000);

  thread_track_ = std::thread(&FrontEnd::startTracking, this);

}

void FrontEnd::run() {
  std::pair<
      double,
      std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>
      feature;
  std::pair<double, std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>>
      imu_interval;
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
      imu_interval.second.clear();
      if (!dataflow_ptr_->getPreintIMUData(cur_time_, imu_interval)) {
        continue;
      }
      //INFO("cur_time: ", cur_time_);
      //INFO("imu time: ", imu_interval.first);
      //for (int i = 0; i < imu_interval.second.size(); i++) {
      //  std::cout << std::fixed << "\t" << imu_interval.second[i].first
      //            << std::endl;
      //}
			
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
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
  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
      frame_features;
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

void FrontEnd::clearState() {
  std::unique_lock<std::mutex> lock(buff_mutex_);
  while (!feature_buf_.empty())
    feature_buf_.pop();
  track_img_cnt_ = 0;
	cur_time_=0;
	prev_time_=-1;
	feature_manager_ptr_->clearState();
}

void FrontEnd::pubTrackImage(const cv::Mat &imgTrack, const double t) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg =
      cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
  pub_track_image_.publish(imgTrackMsg);
}

//
// bool FrontEnd::InitGraphOptimizer(const YAML::Node &config_node) {
//  graph_optimizer_type_ =
//      config_node["graph_optimizer_type_"].as<std::string>();
//  optimize_window_size_ =
//      config_node["optimize_window_size"].as<unsigned int>();
//  if (graph_optimizer_type_ == "ceres") {
//    global_graph_optimizer_ptr_ =
//        std::make_shared<CeresGraphOptimizer>(config_node["ceres_param"]);
//  } else {
//    std::cerr << "没有找到与 " << graph_optimizer_type_
//              << " 对应的图优化模式,请检查配置文件" << std::endl;
//    return false;
//  }
//  std::cout << "后端优化选择的优化器为：" << graph_optimizer_type_ <<
//  std::endl
//            << std::endl;
//
//  graph_optimizer_config_.use_preint_restrain =
//      config_node["ceres_param"]["use_preint_restrain"].as<bool>();
//  graph_optimizer_config_.use_height_restrain =
//      config_node["ceres_param"]["use_height_restrain"].as<bool>();
//  graph_optimizer_config_.use_drp_restrain =
//      config_node["ceres_param"]["use_delta_roll_pitch_restrain"].as<bool>();
//  graph_optimizer_config_.use_dh_restrain =
//      config_node["ceres_param"]["use_delta_height_restrain"].as<bool>();
//  graph_optimizer_config_.use_analytic_jacobian =
//      config_node["use_analytic_jacobian"].as<bool>();
//
//  graph_optimizer_config_.delta_height_noise(0) =
//      config_node[graph_optimizer_type_ + "_param"]["delta_height_noise"]
//          .as<float>();
//  graph_optimizer_config_.world_height_noise(0) =
//      config_node[graph_optimizer_type_ + "_param"]["world_height_noise"]
//          .as<float>();
//
//  graph_optimizer_config_.height_change_range(0) =
//      config_node[graph_optimizer_type_ +
//      "_param"]["optimize_height_restrain"]
//          .as<float>();
//  graph_optimizer_config_.height_change_range(1) = 0.0;
//
//  for (int i = 0; i < 2; ++i) {
//    graph_optimizer_config_.delta_roll_pitch_noise(i) =
//        config_node[graph_optimizer_type_ +
//        "_param"]["delta_roll_pitch_noise"]
//                   [i]
//                       .as<float>() *
//        M_PI / 180.0;
//  }
//
//  return true;
//}

// bool FrontEnd::InitLineExtract(const YAML::Node &config_node)
//{
//  std::string method =
//  config_node["line_feature_extraction_method"].as<std::string>(); if (method
//  == "region_grow") {
//    line_extract_ptr_ =
//    std::make_shared<LineFeatureExtractionRG>(config_node["line_region_grow"]);
//  } else {
//    std::cerr << "没有找到与 " << method << "
//    对应的线段提取方法,请检查配置文件" << std::endl; return false;
//  }
//  std::cout << "前端选择的线段提取方法为：" << method << std::endl <<
//  std::endl;
//
//  return true;
//}
//
// bool FrontEnd::InitRegistration(
//    std::shared_ptr<RegistrationInterface> &registration_ptr,
//    const YAML::Node &config_node) {
//  std::string registration_method =
//      config_node["registration_method"].as<std::string>();
//  std::cout << "前端选择的点云匹配方式为：" << registration_method <<
//  std::endl;
//
//  if (registration_method == "NDT") {
//    registration_ptr =
//        std::make_shared<NDTRegistration>(config_node[registration_method]);
//  } else if (registration_method == "NDT_OMP") {
//    registration_ptr =
//        std::make_shared<NDTOMPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "ICP") {
//    registration_ptr =
//        std::make_shared<ICPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "ICP_SVD") {
//    registration_ptr =
//        std::make_shared<ICPSVDRegistration>(config_node[registration_method]);
//  } else if (registration_method == "SICP") {
//    registration_ptr =
//        std::make_shared<SICPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "ICP_Normal") {
//    registration_ptr = std::make_shared<ICPNormalRegistration>(
//        config_node[registration_method]);
//  } else if (registration_method == "GICP") {
//    registration_ptr =
//        std::make_shared<GICPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "SGICP") {
//    registration_ptr =
//        std::make_shared<SGICPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "PLICP") {
//    registration_ptr =
//        std::make_shared<PLICPRegistration>(config_node[registration_method]);
//  } else if (registration_method == "PLICP2D") {
//    registration_ptr =
//        std::make_shared<PLICP2DRegistration>(config_node[registration_method]);
//  } else {
//    std::cerr << "没找到与 " << registration_method << "
//    相对应的点云匹配方式!"; return false;
//  }
//
//  return true;
//}
//
// bool FrontEnd::InitFilter(std::string filter_user,
//                          std::shared_ptr<CloudFilterInterface> &filter_ptr,
//                          const YAML::Node &config_node) {
//  std::string filter_method =
//      config_node[filter_user + "_filter"].as<std::string>();
//  std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_method
//            << std::endl;
//
//  if (filter_method == "voxel_filter") {
//    filter_ptr =
//        std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
//#ifdef CUDA_FOUND
//  } else if (filter_method == "voxel_filter_gpu") {
//    if (filter_user == "frame") {
//      filter_ptr = std::make_shared<VoxelFilterGpu>(
//          config_node[filter_method][filter_user],
//          config_node["frame_max_points"].as<unsigned int>());
//    } else {
//      filter_ptr = std::make_shared<VoxelFilterGpu>(
//          config_node[filter_method][filter_user],
//          config_node["frame_max_points"].as<unsigned int>() *
//              config_node["local_frame_num_turning"].as<unsigned int>());
//    }
//#endif
//  } else if (filter_method == "grid_filter") {
//    filter_ptr =
//        std::make_shared<GridFilter>(config_node[filter_method][filter_user]);
//  } else if (filter_method == "no_filter") {
//    filter_ptr = std::make_shared<NoFilter>();
//  } else {
//    std::cerr << "没有为 " << filter_user << " 找到与 " << filter_method
//              << " 相对应的滤波方法!";
//    return false;
//  }
//
//  return true;
//}

/*
bool FrontEnd::Update(const CloudData &cloud_data,
                      Eigen::Isometry3f &cloud_pose, PoseData &gnss_pose_data,
                      PoseData &preint_pose_data) {
  // TicToc t;
  if (preint_pose_data.ins_status == 0) {
    ERROR("[FrontEnd][Update] preint_pose_data status is zero!");
    return false;
  }
  static bool local_map_full = false;

  local_map_full = local_map_size_ >= local_frame_num_straight_;
  if (!local_map_full) {
    key_frame_distance_ = 0.1;
    key_frame_angle_ = 1.0;
  } else {
    key_frame_distance_ = key_frame_distance_normal_;
    key_frame_angle_ = key_frame_angle_normal_;
  }

  current_frame_.cloud_data.time = cloud_data.time;
  current_frame_.cloud_data.cloud_ptr = cloud_data.cloud_ptr;
  current_frame_.cloud_data.valid = cloud_data.valid;

  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  if (current_frame_.cloud_data.valid) {
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr,
                              filtered_cloud_ptr);
  } else {
    filtered_cloud_ptr->clear();
  }
  // REMIND("[FrontEnd] frame_filter_ptr_ run time ", t.toc() * 1000, " ms!\n");
  // *filtered_cloud_ptr = *current_frame_.cloud_data.cloud_ptr;

  // TODO
  // static TicToc time;
  // time.tic();
  // std::vector<LineFeature> lines;
  // line_extract_ptr_->Extract(filtered_cloud_ptr, lines);
  // REMIND("[front end] line extraction time is ",time.ave_toc()*1000);

  static Eigen::Isometry3f step_pose =
      Eigen::Isometry3f::Identity(); //相邻两帧间的运动增量
  static float last_key_frame_yaw = 0;
  static Eigen::Matrix<float, 6, 6> preint_cov =
      Eigen::Matrix<float, 6, 6>::Zero();

  // 局部地图容器中没有关键帧，代表是第一帧数据
  // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
  if (local_map_size_ == 0) {
    last_pose_ = init_pose_;
    predict_pose_ = init_pose_;
    current_frame_.pose = init_pose_;
    current_frame_.score = 0.0001;
    current_frame_.yaw = last_key_frame_yaw;
    current_key_gnss_ = gnss_pose_data;
    cloud_pose = current_frame_.pose;

    current_key_preint_ = preint_pose_data;
    current_key_preint_.pose = gnss_pose_data.pose;
    preint_cov = preint_pose_data.covariance;
    current_frame_.preint_pose = current_key_preint_;

    DEBUG("gnss pose\n", current_key_gnss_.pose.matrix());
    odom_init_pose_ =
        current_key_gnss_.pose *
        current_frame_.pose.inverse(); // transform about odom to gnss
    DEBUG("odom pose\n", odom_init_pose_.matrix());
    {
      Eigen::Vector3f rpy = Converter::toEulerAngle(odom_init_pose_.linear());
      Eigen::Vector3f trans = odom_init_pose_.translation();
      INFO("origin rpy   ", rpy.transpose());
      INFO("origin trans ", trans.transpose());
    }
    UpdateWithNewFrame(current_frame_);
    cloud_pose = odom_init_pose_ * cloud_pose; // transform about odom to gnss
    DEBUG("cloud pose\n", cloud_pose.matrix());

    AddKeyframe2Graph();

    current_frame_.id++;

    return true;
  }

  Eigen::Isometry3f tmp_pose = current_key_preint_.pose;
  current_key_preint_ = preint_pose_data;
  current_key_preint_.pose = tmp_pose * preint_pose_data.pose;
  IncRTFromLastFrame_ = preint_pose_data.pose;
  preint_cov += preint_pose_data.covariance;
  current_frame_.preint_pose = current_key_preint_;
  current_frame_.preint_pose.covariance = preint_cov;

  if (current_frame_.cloud_data.valid) {
    current_frame_.cloud_data.valid = Scan2MapMatching(filtered_cloud_ptr);
  } else {
    if (use_fusion_velocity_ && IncRTFromLastFrame_(0, 3) != 0.0) {
      current_frame_.pose = last_pose_ * IncRTFromLastFrame_;
    } else {
      current_frame_.pose = predict_pose_;
    }
  }

  // REMIND("[FrontEnd] Scan2MapMatching run time ", t.toc() * 1000, " ms!\n");
  // TODO
  // if (!PoseFusion()) {
  //  // ERROR("[FrontEnd] Graph optimization failure!");
  //  return false;
  //}

  // 利用帧间判断是否转弯
  CheckTurningState();
  // 根据匀速模型预测下一帧位姿
  step_pose = last_pose_.inverse() * current_frame_.pose;
  predict_pose_ = current_frame_.pose * step_pose;
  last_pose_ = current_frame_.pose;
  // TO DO 通过GNSS序列求解坐标系变换矩阵，降低单次测量噪声的影响
  // UpdateInitMap2WorldTF(current_frame_.pose.topRightCorner(3,1),
  // gnss_pose_data.pose.topRightCorner(3,1));

  // transform laser odom to UTM gnss
  cloud_pose = odom_init_pose_ * current_frame_.pose;

  // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
  bool distance_check =
      (fabs(last_key_frame_pose_(0, 3) - current_frame_.pose(0, 3)) +
       fabs(last_key_frame_pose_(1, 3) - current_frame_.pose(1, 3)) +
       fabs(last_key_frame_pose_(2, 3) - current_frame_.pose(2, 3))) >
              key_frame_distance_
          ? true
          : false;

  float diff = fabs(current_frame_.yaw - last_key_frame_yaw);
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;
  }

  bool angle_check = (diff / M_PI * 180) > key_frame_angle_ ? true : false;

  if (distance_check || angle_check) {

    current_key_gnss_ = gnss_pose_data;
    UpdateWithNewFrame(current_frame_);

    AddKeyframe2Graph();

    last_key_frame_pose_ = current_frame_.pose;
    last_key_frame_yaw = current_frame_.yaw;
    preint_cov.setZero();
  }
  current_frame_.id++;

  // REMIND("[FrontEnd] Other run time ", t.toc() * 1000, " ms!\n");
  return true;
}

bool FrontEnd::Scan2MapMatching(const CloudData::CLOUD_PTR filtered_cloud_ptr) {
  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());

  bool ok = false;
  UpdateFiteredLocalMap();

  std::ofstream fp;
  fp.open("/home/sun/time.txt", std::ios::app);
  static TicToc time;
  time.tic();
  if (use_fusion_velocity_ && IncRTFromLastFrame_(0, 3) != 0.0) {
    ok = registration_ptr_->ScanMatch(filtered_cloud_ptr,
                                      last_pose_ * IncRTFromLastFrame_,
                                      result_cloud_ptr, current_frame_.pose);
  } else {
    ok = registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose_,
                                      result_cloud_ptr, current_frame_.pose);
  }
  fp << time.toc() * 1000.0 << std::endl;
  fp.close();
  // REMIND("[front end ]registration time is ",t.ave_toc()*1000);

  if (ok) {
    // current_frame_.score = registration_ptr_->GetFitnessScore();
    //		DEBUG("[FrontEnd][FitnessScore]
    //GetFitnessScore()=",current_frame_.score);
    current_frame_.score = registration_ptr_->GetRegistEvaluation(
        registration_ptr_->GetInputTarget(), filtered_cloud_ptr,
        current_frame_.pose);
    //		DEBUG("[FrontEnd][FitnessScore]
    //GetRegistEvaluation()=",current_frame_.score);
    // DEBUG("[FrontEnd][ScanMatching] fitnessscore = ", current_frame_.score);
    current_frame_.covariance = registration_ptr_->GetCovariance();
    if (registration_ptr_->DetectDegeneration()) {
      // getchar();
      current_frame_.pose = last_pose_ * IncRTFromLastFrame_;
    }
    // DEBUG("[FrontEnd][ScanMatching] covariance = \n",
    // current_frame_.covariance);
  } else {
    current_frame_.pose = last_pose_ * IncRTFromLastFrame_;
    current_frame_.score = 1.0;
    current_frame_.covariance.setIdentity();
  }

  // DEBUG("current_frame_.covariance=\n", current_frame_.covariance);

  current_frame_.yaw = Converter::toEulerYawAngle(current_frame_.pose.linear());
  while (current_frame_.yaw > M_PI) {
    current_frame_.yaw = current_frame_.yaw - 2 * M_PI;
  }
  while (current_frame_.yaw < -M_PI) {
    current_frame_.yaw = current_frame_.yaw + 2 * M_PI;
  }

  // DEBUG("ok=", ok);

  return ok;
}

bool FrontEnd::PoseFusion() {
  global_graph_optimizer_ptr_->AddSe3Node(current_frame_.pose,
                                          current_frame_.id, false);
  bool ok = false;
  if (AddGraphEdges()) {
    ok = global_graph_optimizer_ptr_->Optimize(
        graph_optimizer_config_.use_analytic_jacobian);
    if (ok)
      current_frame_.pose =
          global_graph_optimizer_ptr_->GetOptimizedPose(current_frame_.id);
  }
  global_graph_optimizer_ptr_->RemoveSe3Node(current_frame_.id);

  return ok;
}

bool FrontEnd::AddKeyframe2Graph() {
  global_graph_optimizer_ptr_->AddSe3Node(current_frame_.pose,
                                          current_frame_.id, true);
  keyframes_opt_.push_back(current_frame_);
  if (keyframes_opt_.size() > optimize_window_size_) {
    global_graph_optimizer_ptr_->RemoveSe3Node(keyframes_opt_.front().id);
    keyframes_opt_.pop_front();
  }
  return true;
}

bool FrontEnd::AddGraphEdges() {
  if (global_graph_optimizer_ptr_->GetNodeNum() < 2)
    return false;

  // absolute height constraint
  if (graph_optimizer_config_.use_height_restrain) {
    global_graph_optimizer_ptr_->AddSe3PriorZEdge(
        current_frame_.id, graph_optimizer_config_.height_change_range,
        graph_optimizer_config_.world_height_noise);
  }

  float delta = 1.0 / keyframes_opt_.size();
  int count = 1;
  for (FrameDeque::iterator it = keyframes_opt_.begin();
       it != keyframes_opt_.end(); ++it) {
    it->weight = delta * count;
    count++;
  }
  for (FrameDeque::const_iterator it = keyframes_opt_.begin();
       it != keyframes_opt_.end(); ++it) {
    // odometry constraint
    if (current_frame_.cloud_data.valid && it->cloud_data.valid) {
      // Eigen::Matrix<float, 6, 6> cov = (current_frame_.covariance +
      // it->covariance) * it->weight; WARNING("odom edge\t" +
      // std::to_string(it->id) + "\t" + std::to_string(current_frame_.id));
      // WARNING("weight = ", it->weight);
      // WARNING(current_frame_.covariance.diagonal().transpose());
      global_graph_optimizer_ptr_->AddSe3Edge(
          it->id, current_frame_.id, it->pose.inverse() * current_frame_.pose,
          (current_frame_.covariance + it->covariance) * it->weight);
    }

    // pre-integral constraint
    if (graph_optimizer_config_.use_preint_restrain && use_fusion_velocity_) {
      Eigen::Matrix<float, 6, 6> cov = current_frame_.preint_pose.covariance;
      for (FrameDeque::const_iterator i = std::next(it);
           i != keyframes_opt_.end(); ++i) {
        cov += i->preint_pose.covariance;
      }
      // WARNING("preint edge\t" + std::to_string(it->id) + "\t" +
      // std::to_string(current_frame_.id));
      // WARNING(cov.diagonal().transpose());
      global_graph_optimizer_ptr_->AddSe3PreintEdge(
          it->id, current_frame_.id,
          it->preint_pose.pose.inverse() * current_frame_.preint_pose.pose,
          cov);
      // current_frame_.preint_pose.covariance + it->preint_pose.covariance);
    }

    // relative height constraint
    if (graph_optimizer_config_.use_dh_restrain) {
      double delta_z = 0.0;
      global_graph_optimizer_ptr_->AddSe3PriordZEdge(
          it->id, current_frame_.id, delta_z,
          graph_optimizer_config_.delta_height_noise);
    }
    // relative orientation constraint
    if (graph_optimizer_config_.use_drp_restrain) {
      Eigen::Vector2f delta_rp = Eigen::Vector2f::Zero();
      global_graph_optimizer_ptr_->AddSe3PriordRPEdge(
          it->id, current_frame_.id, delta_rp,
          graph_optimizer_config_.delta_roll_pitch_noise);
    }
  }

  return true;
}

bool FrontEnd::UpdateInitMap2WorldTF(Eigen::Vector3f odom_trans,
                                     Eigen::Vector3f gnss_trans) {
  // TODO
  //
添加GNSS序列初始化vision_odom坐标系与UTM坐标系的转换矩阵，避免由于单次测量误差引入的坐标系差异过大的问题。
  return true;
}

bool FrontEnd::CheckTurningState() {
  static float last_yaw = current_frame_.yaw;
  static std::deque<bool> turning_states;
  static int turning_filter_size = 5;

  float diff = fabs(current_frame_.yaw - last_yaw);
  last_yaw = current_frame_.yaw;
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;
  }

  bool turning_state =
      (diff / M_PI * 180) > turning_check_angle_ ? true : false;
  turning_states.push_front(turning_state);
  turning_states.resize(turning_filter_size, false);

  turning_indicator_ = true;
  for (int i = 0; i < turning_filter_size; i++) {
    turning_indicator_ = turning_indicator_ && turning_states.at(i);
    if (!turning_indicator_) {
      break;
    }
  }
  return true;
}

bool FrontEnd::SetInitPose(const Eigen::Isometry3f &init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame &new_key_frame) {
  AddNewKeyFrame(new_key_frame);
  Frame key_frame = new_key_frame;
  // 这一步的目的是为了把关键帧的点云保存下来
  // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
  // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
  if (key_frame.cloud_data.valid) {
    key_frame.cloud_data.cloud_ptr.reset(
        new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    pcl::transformPointCloud(*key_frame.cloud_data.cloud_ptr,
                             *transformed_cloud_ptr, key_frame.pose);
    *local_map_ptr_ += *transformed_cloud_ptr;
  }
  // 更新局部地图
  local_map_size_++;
  local_map_points_number_.push_back(key_frame.cloud_data.cloud_ptr->size());
  UpdateLocalMap();
  if (local_map_ptr_->size() < 10)
    local_map_valid = false;

  SetLocalMapFilterInput();

  return true;
}

bool FrontEnd::UpdateLocalMap() {
  unsigned int delete_number = 0;
  if (!turning_indicator_) {
    if (local_map_points_number_.size() == local_frame_num_straight_ + 1) {
      delete_number = local_map_points_number_[0];

      local_map_points_number_.pop_front();
      local_map_size_ -= 1;
    } else if (local_map_points_number_.size() >
               local_frame_num_straight_ + 1) {
      delete_number = local_map_points_number_[0] + local_map_points_number_[1];
      local_map_points_number_.erase(local_map_points_number_.begin(),
                                     local_map_points_number_.begin() + 2);
      local_map_size_ -= 2;
    }
  } else {
    if ((local_map_points_number_.size() > local_frame_num_straight_) &&
        (local_map_points_number_.size() <= local_frame_num_turning_)) {
      delete_number = 0;
    } else if (local_map_points_number_.size() > local_frame_num_turning_) {
      delete_number = local_map_points_number_[0];

      local_map_points_number_.pop_front();
      local_map_size_ -= 1;
    }
  }
  if (delete_number > 0) {
    local_map_ptr_->erase(local_map_ptr_->begin(),
                          local_map_ptr_->begin() + delete_number);
  }
  return true;
}

bool FrontEnd::SetLocalMapFilterInput() {
  if (!local_map_valid) {
    return false;
  }

  if (local_map_size_ < local_frame_num_straight_) {
    NoFilter filter_ptr;
    CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
    filter_ptr.Filter(local_map_ptr_, filtered_local_map_ptr);
    registration_ptr_->SetInputTarget(filtered_local_map_ptr);

    filter_thread_ptr_->SetInputCloud(local_map_ptr_);
  } else {
    // local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr_);

    filter_thread_ptr_->SetInputCloud(local_map_ptr_);
  }

  return true;
}
bool FrontEnd::UpdateFiteredLocalMap() {
  if (local_map_size_ < local_frame_num_straight_)
    return false;

  if (filter_thread_ptr_->GetFilteredCloud(filtered_local_map_ptr_)) {
    registration_ptr_->SetInputTarget(filtered_local_map_ptr_);
  }
  return true;
}

bool FrontEnd::AddNewKeyFrame(const Frame &new_key_frame) {
  // 把关键帧点云存储到硬盘里
  if (new_key_frame.cloud_data.valid) {
    std::string file_path =
        key_frames_path_ + "/key_frame_" + std::to_string(mnKFId) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);
  }

  CurrentKF_.time = new_key_frame.cloud_data.time;
  CurrentKF_.index = mnKFId;
  CurrentKF_.pose = odom_init_pose_ * new_key_frame.pose; // 转换到相对map系中。
  CurrentKF_.covariance = new_key_frame.covariance;

  mnKFId++;
  has_new_key_frame_ = true;

  return true;
}

KeyFrame FrontEnd::GetNewKF() { return CurrentKF_; }

PoseData FrontEnd::GetNewKeyGnss() { return current_key_gnss_; }
PoseData FrontEnd::GetNewKeyPreint() { return current_key_preint_; }

*/

} // namespace vslam
