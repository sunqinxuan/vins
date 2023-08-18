/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 14:41
#
# Filename:		vio.cpp
#
# Description:
#
************************************************/

#include "vio/vio.hpp"

namespace vslam {
VIOdometry::VIOdometry(ros::NodeHandle &nh, std::string config_file_path) {

  std::string config_file = config_file_path + "config.yaml";
  cv::FileStorage config(config_file, cv::FileStorage::READ);

  std::string cam0_file, cam1_file;
  config["cam0_calib"] >> cam0_file;
  config["cam1_calib"] >> cam1_file;
  std::string cam0_file_path = config_file_path + cam0_file;
  std::string cam1_file_path = config_file_path + cam1_file;
  std::cout << "cam0_file_path: " << cam0_file_path << std::endl;
  std::cout << "cam1_file_path: " << cam1_file_path << std::endl;

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

  double focal_length_ = config["focal_length"];
  double min_parallax = config["keyframe_parallax"];
  min_parallax = min_parallax / focal_length_;

  int num_iter = config["max_num_iterations"];
  double max_time = config["max_solver_time"];
  bool optimizer_debug; // = config["minimizer_progress_to_stdout"];
  config["minimizer_progress_to_stdout"] >> optimizer_debug;

  config["estimate_extrinsic"] >> estimate_extrinsic_;
  DEBUG("estimate_extrinsic: ", estimate_extrinsic_);

  config.release();

  dt_buf_.resize(window_size_ + 1);
  imu_meas_buf_.resize(window_size_ + 1);

  imu_preint_ptr_ =
      std::make_shared<IMUPreintegration>(acc_n, gyr_n, acc_w, gyr_w, gravity);
  imu_preintegrations_.resize(window_size_ + 1);
  for (auto &imu_preint : imu_preintegrations_) {
    imu_preint = std::make_shared<IMUPreintegration>(acc_n, gyr_n, acc_w, gyr_w,
                                                     gravity);
  }
  // marginalization_info_ = std::make_shared<MarginalizationInfo>();

  feature_manager_ptr_ = std::make_shared<FeatureManager>(
      focal_length_, min_parallax, window_size_);

  clearState();

  dataflow_ptr_ = std::make_shared<DataFlow>(nh, config_file_path);
  dataflow_ptr_->start();

  feature_tracker_ptr_ =
      std::make_shared<FeatureTracker>(max_cnt, min_dist, F_thres);
  feature_tracker_ptr_->readCameraIntrinsics(cam0_file_path, cam1_file_path);

  initial_alignment_ptr_ = std::make_shared<InitialAlignment>();

  graph_optimizer_ptr_ = std::make_shared<FactorGraphOptimizer>(
      num_iter, max_time, optimizer_debug, focal_length_ / 1.5, window_size_,
      dataflow_ptr_->useIMU());

  initial_ex_rotation_ptr_ = std::make_shared<InitialEXRotation>(window_size_);

  thread_track_ = std::thread(&VIOdometry::startTracking, this);
}

void VIOdometry::clearState() {
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
  // TODO
  marginalization_param_blocks_.clear();

  open_ex_estimation_ = false;
}

void VIOdometry::run() {
  std::pair<double, PointsTrack> feature;
  std::vector<IMUMeasureTime> imu_interval;
  while (true) {
    // if (flag_)
    //  break;

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
      if (dataflow_ptr_->useIMU()) {
        imu_interval.clear();
        if (!dataflow_ptr_->getIMUInterval(prev_time_, cur_time_,
                                           imu_interval)) {
          continue;
        }
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

      if (dataflow_ptr_->useIMU()) {
        // process IMU
        if (!init_first_pose_flag_) {
          initIMUPose(imu_interval);
        }
        preintIMU(imu_interval);
      }

      //			prog_mutex_.lock();
      // process image
      processFrame(feature.first, feature.second);

      prev_time_ = cur_time_;

      // TODO:printStatistics()

      dataflow_ptr_->pubOdometry(feature.first, nav_states_[window_size_],
                                 is_initialized_flag_);
      dataflow_ptr_->pubKeyPoses(feature.first, nav_states_);
      dataflow_ptr_->pubCameraPose(feature.first, nav_states_[window_size_ - 1],
                                   Tic0_calib_, Tic1_calib_,
                                   is_initialized_flag_);
      dataflow_ptr_->pubPointCloud(feature.first, nav_states_, Tic0_calib_,
                                   feature_manager_ptr_->feature);
      dataflow_ptr_->pubKeyframe(time_stamps_[window_size_ - 2], nav_states_,
                                 Tic0_calib_, feature_manager_ptr_->feature,
                                 is_initialized_flag_, margin_old_);
      dataflow_ptr_->pubTF(feature.first, nav_states_, Tic0_calib_,
                           is_initialized_flag_);
      //			prog_mutex_.unlock();

      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
}

void VIOdometry::preintIMU(const std::vector<IMUMeasureTime> &imu_intv) {
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

void VIOdometry::processFrame(const double &time, const PointsTrack &points) {
  printf("new image coming ------------------------------------------\n");
  printf("Adding feature points %lu\n", points.size());

  if (feature_manager_ptr_->addFeatureCheckParallax(frame_cnt_, points, 0.0)) {
    margin_old_ = true;
  } else {
    margin_old_ = false;
  }
  INFO("margin_old_: ", margin_old_);
  INFO("num of features: ", feature_manager_ptr_->getFeatureCount());
  time_stamps_[frame_cnt_] = time;

  ImageFrame imgframe(points, time);
  imgframe.imu_preint_ptr_ = imu_preint_ptr_->clone();
  image_frames_.insert(std::make_pair(time, imgframe));
  imu_preint_ptr_->reset(imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
                         imu_bias_[frame_cnt_].head<3>(),
                         imu_bias_[frame_cnt_].tail<3>());

  if (estimate_extrinsic_ == ESTIMATE) {
    REMIND("calibrating extrinsic parameters, please rotate the camera-IMU "
           "rig...");
    if (frame_cnt_ != 0) {
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres =
          feature_manager_ptr_->getCorresponding(frame_cnt_ - 1, frame_cnt_);
      Eigen::Matrix3d calib_ric;
      if (initial_ex_rotation_ptr_->CalibrationExRotation(
              corres, imu_preintegrations_[frame_cnt_]->getDeltaQ(),
              calib_ric)) {
        REMIND("initial extrinsic rotation success!");
        REMIND("initial extrinsic rotation: ");
        REMIND(calib_ric);
        Tic0_calib_.linear() = calib_ric;
        estimate_extrinsic_ = OPTIMIZE;
      }
    }
  }

  if (!is_initialized_flag_) {
    WARNING(nav_states_[frame_cnt_]);
    REMIND(Tic0_calib_.matrix());
    if (dataflow_ptr_->useIMU()) {
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
        updateLatestStates();
        is_initialized_flag_ = true;
        slideWindow();
        INFO("Initialization finish!");
      }
    } else {
      feature_manager_ptr_->initFramePoseByPnP(frame_cnt_, nav_states_,
                                               Tic0_calib_);
      feature_manager_ptr_->triangulate(nav_states_, Tic0_calib_, Tic1_calib_);
      optimize();

      if (frame_cnt_ == window_size_) {
        optimize();
        updateLatestStates();
        is_initialized_flag_ = true;
        slideWindow();
        INFO("Initialization finish!");
      }
    }

    if (frame_cnt_ < window_size_) {
      frame_cnt_++;
      int prev = frame_cnt_ - 1;
      nav_states_[frame_cnt_] = nav_states_[prev];
      imu_bias_[frame_cnt_] = imu_bias_[prev];
    }
  } else { // if(!is_initialized_flag_)
    if (!dataflow_ptr_->useIMU()) {
      feature_manager_ptr_->initFramePoseByPnP(frame_cnt_, nav_states_,
                                               Tic0_calib_);
    }
    feature_manager_ptr_->triangulate(nav_states_, Tic0_calib_, Tic1_calib_);
    optimize();
    // flag_ = true;

    std::set<int> rm_idx;
    detectOutliers(rm_idx);
    feature_manager_ptr_->removeOutlier(rm_idx);

    slideWindow();
    feature_manager_ptr_->removeFailures();

    updateLatestStates();
  }
}

void VIOdometry::detectOutliers(std::set<int> &rm_idx) {
  int feature_index = -1;
  for (auto &it_per_id : feature_manager_ptr_->feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Eigen::Vector3d pts_j = it_per_frame.point;
        // double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0],
        // tic[0], Rs[imu_j], Ps[imu_j], ric[0], tic[0], depth, pts_i, pts_j);
        double tmp_error =
            reprojectionError(nav_states_[imu_i], nav_states_[imu_j],
                              Tic0_calib_, Tic0_calib_, depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
      }
      // need to rewrite projecton factor.........
      if (it_per_frame.is_stereo) {

        Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          // double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0],
          // tic[0], Rs[imu_j], Ps[imu_j], ric[1], tic[1], depth, pts_i,
          // pts_j_right);
          double tmp_error = reprojectionError(
              nav_states_[imu_i], nav_states_[imu_j], Tic0_calib_, Tic1_calib_,
              depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        } else {
          // double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0],
          // tic[0], Rs[imu_j], Ps[imu_j], ric[1], tic[1], depth, pts_i,
          // pts_j_right);
          double tmp_error = reprojectionError(
              nav_states_[imu_i], nav_states_[imu_j], Tic0_calib_, Tic1_calib_,
              depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        }
      }
    }
    double ave_err = err / errCnt;
    if (ave_err * focal_length_ > 3)
      rm_idx.insert(it_per_id.feature_id);
  }
}

double VIOdometry::reprojectionError(const NavState &nsi, const NavState &nsj,
                                     const Eigen::Isometry3d &Tici,
                                     const Eigen::Isometry3d &Ticj,
                                     double depth, const Eigen::Vector3d &uvi,
                                     const Eigen::Vector3d &uvj) {
  Eigen::Vector3d pts_w =
      nsi.rotation() * (Tici.linear() * (depth * uvi) + Tici.translation()) +
      nsi.position();
  Eigen::Vector3d pts_cj =
      Ticj.linear().transpose() *
      (nsj.rotation().transpose() * (pts_w - nsj.position()) -
       Ticj.translation());
  // Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  // Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) -
  // ticj);
  Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void VIOdometry::optimize() {

  graph_optimizer_ptr_->initGraph();

  Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
  if (dataflow_ptr_->useIMU()) {
    graph_optimizer_ptr_->setVertices(nav_states_, imu_bias_, Tic0_calib_,
                                      Tic1_calib_, depth_vec);
  } else {
    graph_optimizer_ptr_->setVertices(nav_states_, Tic0_calib_, Tic1_calib_,
                                      depth_vec);
  }

  for (int i = 0; i < frame_cnt_ + 1; i++) {
    if (!dataflow_ptr_->useIMU() && i == 0)
      graph_optimizer_ptr_->addParamNavState(i, true);
    else
      graph_optimizer_ptr_->addParamNavState(i);
  }
  if ((estimate_extrinsic_ && frame_cnt_ == window_size_ &&
       nav_states_[0].velocity().norm() > 0.2) ||
      open_ex_estimation_) {
    open_ex_estimation_ = true;
    graph_optimizer_ptr_->addParamCameraEx(0);
    graph_optimizer_ptr_->addParamCameraEx(1);
  } else {
    graph_optimizer_ptr_->addParamCameraEx(0, true);
    graph_optimizer_ptr_->addParamCameraEx(1, true);
  }

  // TODO
  // if(marginalization_info_)
  DEBUG("marginalization_info_==nullptr : ", marginalization_info_ == nullptr);
  if (marginalization_info_ != nullptr)
    graph_optimizer_ptr_->addFactorMargin(marginalization_info_,
                                          marginalization_param_blocks_);
  DEBUG("addFactorMargin");

  if (dataflow_ptr_->useIMU()) {
    for (int i = 0; i < frame_cnt_; i++) {
      int j = i + 1;
      graph_optimizer_ptr_->addFactorIMU(i, j, imu_preintegrations_[j]);
    }
  }

  // Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
  // for (int i = 0; i < feature_manager_ptr_->getFeatureCount(); i++) {
  //  graph_optimizer_ptr_->addParamFeature(i, depth_vec(i));
  //}

  // std::fstream fp;
  // fp.open("/home/sun/vins_sqx/debug.txt", std::ios::out);

  int f_m_cnt = 0;
  int feature_index = -1;
  for (auto &it_per_id : feature_manager_ptr_->feature) {
    // fp << std::endl
    //   << f_m_cnt << "  **************************************************"
    //   << std::endl;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;
    // fp << "it_per_id.used_num = " << it_per_id.used_num << std::endl;

    ++feature_index;
    // fp << "feature_index = " << feature_index << std::endl;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    // fp << "imu_i = " << imu_i << std::endl;

    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    // fp << "pts_i = " << pts_i.transpose() << std::endl;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point;
      Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
      // fp << "\timu_j = " << imu_j << std::endl;
      // fp << "\tpts_j = " << pts_j.transpose() << std::endl;
      // fp << "\tpts_j_right = " << pts_j_right.transpose() << std::endl;
      // fp << "\tit_per_id.feature_per_frame[0].velocity = "
      //   << it_per_id.feature_per_frame[0].velocity.transpose() << std::endl;
      // fp << "\tit_per_frame.velocity = " << it_per_frame.velocity.transpose()
      //   << std::endl;
      // fp << "\tit_per_frame.velocityRight = "
      //   << it_per_frame.velocityRight.transpose() << std::endl;
      // fp << "\tit_per_id.feature_per_frame[0].cur_td = "
      //   << it_per_id.feature_per_frame[0].cur_td << std::endl;
      // fp << "\tit_per_frame.cur_td = " << it_per_frame.cur_td << std::endl;
      if (imu_i != imu_j) {
        // fp << "\t\t=============add 21 ================" << std::endl;
        graph_optimizer_ptr_->addFactorProject21(
            imu_i, imu_j, feature_index, pts_i, pts_j,
            it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
        // fp << "\t\tpose[imu_i] = "
        //   <<
        //   graph_optimizer_ptr_->vertices_nav_state_[imu_i].pose.transpose()
        //   << std::endl;
        // fp << "\t\tpose[imu_j] = "
        //   <<
        //   graph_optimizer_ptr_->vertices_nav_state_[imu_j].pose.transpose()
        //   << std::endl;
        // fp << "\t\tcam_ex[0] = "
        //   << graph_optimizer_ptr_->vertices_camera_ex_[0].pose.transpose()
        //   << std::endl;
        // fp << "\t\tfeature[f_idx] = "
        //   << graph_optimizer_ptr_->vertices_feature_[feature_index].depth(0)
        //   << std::endl;
        // fp << "\t\ttd = " << graph_optimizer_ptr_->vertices_td_(0) <<
        // std::endl;
        // ProjectionTwoFrameOneCamFactor *f_td =
        //    new ProjectionTwoFrameOneCamFactor(
        //        pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
        //        it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
        //        it_per_frame.cur_td);
        // graph_optimizer_ptr_->problem_->AddResidualBlock(
        //    f_td, graph_optimizer_ptr_->loss_function_,
        //    graph_optimizer_ptr_->vertices_nav_state_[imu_i].pose.data(),
        //    graph_optimizer_ptr_->vertices_nav_state_[imu_j].pose.data(),
        //    graph_optimizer_ptr_->vertices_camera_ex_[0].pose.data(),
        //    graph_optimizer_ptr_->vertices_feature_[feature_index].depth.data(),
        //    graph_optimizer_ptr_->vertices_td_.data());
      }
      if (it_per_frame.is_stereo) {
        if (imu_i != imu_j) {
          // fp << "\t\t=============add 22 ================" << std::endl;
          graph_optimizer_ptr_->addFactorProject22(
              imu_i, imu_j, feature_index, pts_i, pts_j_right,
              it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          // fp << "\t\tpose[imu_i] = "
          //   << graph_optimizer_ptr_->vertices_nav_state_[imu_i]
          //          .pose.transpose()
          //   << std::endl;
          // fp << "\t\tpose[imu_j] = "
          //   << graph_optimizer_ptr_->vertices_nav_state_[imu_j]
          //          .pose.transpose()
          //   << std::endl;
          // fp << "\t\tcam_ex[0] = "
          //   << graph_optimizer_ptr_->vertices_camera_ex_[0].pose.transpose()
          //   << std::endl;
          // fp << "\t\tcam_ex[1] = "
          //   << graph_optimizer_ptr_->vertices_camera_ex_[1].pose.transpose()
          //   << std::endl;
          // fp << "\t\tfeature[f_idx] = "
          //   <<
          //   graph_optimizer_ptr_->vertices_feature_[feature_index].depth(0)
          //   << std::endl;
          // fp << "\t\ttd = " << graph_optimizer_ptr_->vertices_td_(0)
          //   << std::endl;
          // ProjectionTwoFrameTwoCamFactor *f =
          //    new ProjectionTwoFrameTwoCamFactor(
          //        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
          //        it_per_frame.velocityRight,
          //        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          // graph_optimizer_ptr_->problem_->AddResidualBlock(
          //    f, graph_optimizer_ptr_->loss_function_,
          //    graph_optimizer_ptr_->vertices_nav_state_[imu_i].pose.data(),
          //    graph_optimizer_ptr_->vertices_nav_state_[imu_j].pose.data(),
          //    graph_optimizer_ptr_->vertices_camera_ex_[0].pose.data(),
          //    graph_optimizer_ptr_->vertices_camera_ex_[1].pose.data(),
          //    graph_optimizer_ptr_->vertices_feature_[feature_index]
          //        .depth.data(),
          //    graph_optimizer_ptr_->vertices_td_.data());
        } else {
          // fp << "\t\t=============add 12 ================" << std::endl;
          graph_optimizer_ptr_->addFactorProject12(
              feature_index, pts_i, pts_j_right,
              it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          // fp << "\t\tcam_ex[0] = "
          //   << graph_optimizer_ptr_->vertices_camera_ex_[0].pose.transpose()
          //   << std::endl;
          // fp << "\t\tcam_ex[1] = "
          //   << graph_optimizer_ptr_->vertices_camera_ex_[1].pose.transpose()
          //   << std::endl;
          // fp << "\t\tfeature[f_idx] = "
          //   <<
          //   graph_optimizer_ptr_->vertices_feature_[feature_index].depth(0)
          //   << std::endl;
          // fp << "\t\ttd = " << graph_optimizer_ptr_->vertices_td_(0)
          //   << std::endl;
          // ProjectionOneFrameTwoCamFactor *f =
          //    new ProjectionOneFrameTwoCamFactor(
          //        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
          //        it_per_frame.velocityRight,
          //        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          // graph_optimizer_ptr_->problem_->AddResidualBlock(
          //    f, graph_optimizer_ptr_->loss_function_,
          //    graph_optimizer_ptr_->vertices_camera_ex_[0].pose.data(),
          //    graph_optimizer_ptr_->vertices_camera_ex_[1].pose.data(),
          //    graph_optimizer_ptr_->vertices_feature_[feature_index]
          //        .depth.data(),
          //    graph_optimizer_ptr_->vertices_td_.data());
        }
      }
      f_m_cnt++;
    }
  }
  // fp.close();
  DEBUG("visual measurement count: ", f_m_cnt);

  if (!graph_optimizer_ptr_->optimize()) {
    WARNING("optimization failure!");
  }

  // line1158: Estimator::double2vector();
  updateOptimizedStates();
  DEBUG("updateOptimizedStates");
  graph_optimizer_ptr_->clearGraph();
  DEBUG("clearGraph");

  if (frame_cnt_ < window_size_)
    return;

  // double *pt=graph_optimizer_ptr_->getParamNavState(0);
  // DEBUG("graph_optimizer_ptr_->getParamNavState(0)=",pt);

  if (margin_old_) {
    // std::shared_ptr<MarginalizationInfo> margin_info =
    //    std::make_shared<MarginalizationInfo>();
    graph_optimizer_ptr_->initMarginalization();

    Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
    graph_optimizer_ptr_->setVertices(nav_states_, imu_bias_, Tic0_calib_,
                                      Tic1_calib_, depth_vec);

    DEBUG("marginalization_info_==nullptr : ",
          marginalization_info_ == nullptr);
    if (marginalization_info_ != nullptr && marginalization_info_->valid) {
      std::vector<int> drop_set;
      for (size_t i = 0; i < marginalization_param_blocks_.size(); i++) {
        if (marginalization_param_blocks_[i] ==
                graph_optimizer_ptr_->getParamNavState(0) ||
            marginalization_param_blocks_[i] ==
                graph_optimizer_ptr_->getParamVelBias(0))
          drop_set.push_back(i);
      }
      DEBUG("drop_set:");
      for (int i = 0; i < drop_set.size(); i++)
        std::cout << drop_set[i] << std::endl;
      graph_optimizer_ptr_->addMGFactorMargin(
          marginalization_info_, marginalization_param_blocks_, drop_set);
      DEBUG("addMGFactorMargin");
    }

    if (dataflow_ptr_->useIMU()) {
      graph_optimizer_ptr_->addMGFactorIMU(0, 1, imu_preintegrations_[1]);
      DEBUG("addMGFactorIMU");
    }

    int feature_index = -1;
    for (auto &it_per_id : feature_manager_ptr_->feature) {
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (it_per_id.used_num < 4)
        continue;

      ++feature_index;

      int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
      if (imu_i != 0)
        continue;

      Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

      for (auto &it_per_frame : it_per_id.feature_per_frame) {
        imu_j++;
        Eigen::Vector3d pts_j = it_per_frame.point;
        Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          graph_optimizer_ptr_->addMGFactorProject21(
              imu_i, imu_j, feature_index, pts_i, pts_j,
              it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
              it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
        }
        if (it_per_frame.is_stereo) {
          if (imu_i != imu_j) {
            graph_optimizer_ptr_->addMGFactorProject22(
                imu_i, imu_j, feature_index, pts_i, pts_j_right,
                it_per_id.feature_per_frame[0].velocity,
                it_per_frame.velocityRight,
                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          } else {
            graph_optimizer_ptr_->addMGFactorProject12(
                feature_index, pts_i, pts_j_right,
                it_per_id.feature_per_frame[0].velocity,
                it_per_frame.velocityRight,
                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          }
        }
      }
    }
    DEBUG("addMGFactorProject");

    graph_optimizer_ptr_->marginalize();

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= window_size_; i++) {
      addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamNavState(
          i))] = graph_optimizer_ptr_->getParamNavState(i - 1);
      if (dataflow_ptr_->useIMU()) {
        addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamVelBias(
            i))] = graph_optimizer_ptr_->getParamVelBias(i - 1);
      }
    }
    addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamCameraEx(
        0))] = graph_optimizer_ptr_->getParamCameraEx(0);
    addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamCameraEx(
        1))] = graph_optimizer_ptr_->getParamCameraEx(1);
    addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamTd())] =
        graph_optimizer_ptr_->getParamTd();

    // DEBUG("addr_shift");
    // for (auto it : addr_shift) {
    //  std::cout << it.first << "\t" << it.second << std::endl;
    //}

    std::vector<double *> parameter_blocks =
        graph_optimizer_ptr_->getMarginPtr()->getParameterBlocks(addr_shift);

    // DEBUG("parameter_blocks");
    // for (int i = 0; i < parameter_blocks.size(); i++) {
    //  std::cout << i << "\t" << parameter_blocks[i] << std::endl;
    //}

    // TODO
    marginalization_info_ = graph_optimizer_ptr_->getMarginPtr();
    marginalization_param_blocks_ = parameter_blocks;

  } else {
    if (marginalization_info_ != nullptr &&
        std::count(std::begin(marginalization_param_blocks_),
                   std::end(marginalization_param_blocks_),
                   graph_optimizer_ptr_->getParamNavState(window_size_ - 1))) {
      graph_optimizer_ptr_->initMarginalization();

      Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
      graph_optimizer_ptr_->setVertices(nav_states_, imu_bias_, Tic0_calib_,
                                        Tic1_calib_, depth_vec);

      if (marginalization_info_ != nullptr && marginalization_info_->valid) {
        std::vector<int> drop_set;
        for (size_t i = 0; i < marginalization_param_blocks_.size(); i++) {
          // marginalization_param_blocks_[i]!=graph_optimizer_ptr_->
          if (marginalization_param_blocks_[i] ==
              graph_optimizer_ptr_->getParamNavState(window_size_ - 1))
            drop_set.push_back(i);
        }
        graph_optimizer_ptr_->addMGFactorMargin(
            marginalization_info_, marginalization_param_blocks_, drop_set);
      }

      graph_optimizer_ptr_->marginalize();

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= window_size_; i++) {
        if (i == window_size_ - 1)
          continue;
        else if (i == window_size_) {
          addr_shift[reinterpret_cast<long>(
              graph_optimizer_ptr_->getParamNavState(i))] =
              graph_optimizer_ptr_->getParamNavState(i - 1);
          if (dataflow_ptr_->useIMU()) {
            addr_shift[reinterpret_cast<long>(
                graph_optimizer_ptr_->getParamVelBias(i))] =
                graph_optimizer_ptr_->getParamVelBias(i - 1);
          }
        } else {
          addr_shift[reinterpret_cast<long>(
              graph_optimizer_ptr_->getParamNavState(i))] =
              graph_optimizer_ptr_->getParamNavState(i);
          if (dataflow_ptr_->useIMU()) {
            addr_shift[reinterpret_cast<long>(
                graph_optimizer_ptr_->getParamVelBias(i))] =
                graph_optimizer_ptr_->getParamVelBias(i);
          }
        }
      }
      addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamCameraEx(
          0))] = graph_optimizer_ptr_->getParamCameraEx(0);
      addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamCameraEx(
          1))] = graph_optimizer_ptr_->getParamCameraEx(1);
      addr_shift[reinterpret_cast<long>(graph_optimizer_ptr_->getParamTd())] =
          graph_optimizer_ptr_->getParamTd();

      std::vector<double *> parameter_blocks =
          graph_optimizer_ptr_->getMarginPtr()->getParameterBlocks(addr_shift);

      marginalization_info_ = graph_optimizer_ptr_->getMarginPtr();
      marginalization_param_blocks_ = parameter_blocks;
    }
  }
}

void VIOdometry::updateOptimizedStates() {
  Eigen::Vector3d org_R0 = Converter::R2ypr(nav_states_[0].rotation());
  Eigen::Vector3d org_P0 = nav_states_[0].position();

  if (dataflow_ptr_->useIMU()) {
    NavState nav_state_0;
    graph_optimizer_ptr_->getNavState(0, nav_state_0);
    Eigen::Vector3d org_R00 = Converter::R2ypr(nav_state_0.rotation());

    double d_yaw = org_R0.x() - org_R00.x();
    Eigen::Matrix3d dR0 = Converter::ypr2R(Eigen::Vector3d(d_yaw, 0, 0));
    if (abs(abs(org_R0.y()) - 90) < 1.0 || abs(abs(org_R00.y()) - 90) < 1.0) {
      WARNING("euler singular point!");
      dR0 = nav_states_[0].rotation() * nav_state_0.rotation();
    }

    for (int i = 0; i <= window_size_; i++) {
      graph_optimizer_ptr_->getNavStateBias(i, nav_states_[i], imu_bias_[i],
                                            dR0, org_P0);
    }
  } else {
    for (int i = 0; i <= window_size_; i++) {
      graph_optimizer_ptr_->getNavState(i, nav_states_[i]);
    }
  }

  if (dataflow_ptr_->useIMU()) {
    graph_optimizer_ptr_->getCameraEx(0, Tic0_calib_);
    graph_optimizer_ptr_->getCameraEx(1, Tic1_calib_);
  }

  Eigen::VectorXd depth_vec = feature_manager_ptr_->getDepthVector();
  for (int i = 0; i < feature_manager_ptr_->getFeatureCount(); i++) {
    graph_optimizer_ptr_->getFeature(i, depth_vec(i));
  }
  feature_manager_ptr_->setDepth(depth_vec);
}

void VIOdometry::updateLatestStates() {
  std::unique_lock<std::mutex> lock(prog_mutex_);
  latest_time_ = time_stamps_[frame_cnt_];
  latest_nav_state_ = nav_states_[frame_cnt_];
  latest_imu_bias_ = imu_bias_[frame_cnt_];
  latest_imu_meas_ = imu_meas_0_;

  DEBUG("updateLatestStates");
  std::cout << "latest_time_=" << std::fixed << latest_time_ << std::endl;
  std::cout << "latest_nav_state_=" << std::endl
            << latest_nav_state_ << std::endl;
  std::cout << "latest_imu_bias_=" << latest_imu_bias_.transpose() << std::endl;
  std::cout << "latest_imu_meas_=" << latest_imu_meas_.transpose() << std::endl;

  std::deque<IMUMeasureTime> imubuf = dataflow_ptr_->getIMUBuf();
  DEBUG("imubuf");
  while (!imubuf.empty()) {
    double t = imubuf.front().first;
    Eigen::Vector3d acc = imubuf.front().second.head<3>();
    Eigen::Vector3d gyr = imubuf.front().second.tail<3>();
    std::cout << std::fixed << t << "\t" << acc.transpose() << "\t"
              << gyr.transpose() << std::endl;
    predictIMU(t, acc, gyr);
    imubuf.pop_front();
  }

  DEBUG("updateLatestStates - predictIMU");
  std::cout << "latest_time_=" << std::fixed << latest_time_ << std::endl;
  std::cout << "latest_nav_state_=" << std::endl
            << latest_nav_state_ << std::endl;
  std::cout << "latest_imu_bias_=" << latest_imu_bias_.transpose() << std::endl;
  std::cout << "latest_imu_meas_=" << latest_imu_meas_.transpose() << std::endl;
}

void VIOdometry::slideWindow() {
  if (margin_old_) {
    double t_0 = time_stamps_[0];
    Eigen::Matrix3d back_R0 = nav_states_[0].rotation();
    Eigen::Vector3d back_P0 = nav_states_[0].position();

    // DEBUG("slideWindow");
    // std::cout << "t_0=" << std::fixed << t_0 << std::endl;
    // std::cout << "back_R0=" << std::endl << back_R0 << std::endl;
    // std::cout << "back_P0=" << back_P0.transpose() << std::endl;

    if (frame_cnt_ == window_size_) {
      for (int i = 0; i < window_size_; i++) {
        time_stamps_[i] = time_stamps_[i + 1];
        nav_states_[i].swap(nav_states_[i + 1]);
        if (dataflow_ptr_->useIMU()) {
          imu_bias_[i].swap(imu_bias_[i + 1]);
          dt_buf_[i].swap(dt_buf_[i + 1]);
          imu_meas_buf_[i].swap(imu_meas_buf_[i + 1]);
          std::swap(imu_preintegrations_[i], imu_preintegrations_[i + 1]);
        }
      }

      time_stamps_[window_size_] = time_stamps_[window_size_ - 1];
      nav_states_[window_size_] = nav_states_[window_size_ - 1];
      if (dataflow_ptr_->useIMU()) {
        imu_bias_[window_size_] = imu_bias_[window_size_ - 1];
        dt_buf_[window_size_].clear();
        imu_meas_buf_[window_size_].clear();
        imu_preintegrations_[window_size_]->reset(
            imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
            imu_bias_[window_size_].head<3>(),
            imu_bias_[window_size_].tail<3>());
      }

      // std::ofstream fp;
      // fp.open("/home/sun/vins_sqx/debug.txt", std::ios::out);
      // using namespace std;
      // for (int i = 0; i < window_size_; i++) {
      //  fp << i << " ************************************************" <<
      //  endl; fp << "time=" << fixed << time_stamps_[i] << endl; fp <<
      //  "nav_state=" << endl << nav_states_[i]; fp << "imu_bias_=" <<
      //  imu_bias_[i].transpose() << endl; fp << "dt_buf_: " <<
      //  dt_buf_[i].size() << endl; fp << "imu_meas_buf_:" <<
      //  imu_meas_buf_[i].size() << endl; for (int j = 0; j <
      //  imu_meas_buf_[i].size(); j++) {
      //    fp << "\t" << fixed << dt_buf_[i][j] << "\t"
      //       << imu_meas_buf_[i][j].transpose() << endl;
      //  }
      //}
      // fp.close();

      if (true || !is_initialized_flag_) {
        std::map<double, ImageFrame>::iterator it_0;
        it_0 = image_frames_.find(t_0);
        it_0->second.imu_preint_ptr_->reset();
        image_frames_.erase(image_frames_.begin(), it_0);
      }

      // Estimator::slideWindowOld()
      // TODO: sum_of_back++;
      bool shift_depth = is_initialized_flag_;
      if (shift_depth) {
        Eigen::Matrix3d R0, R1;
        Eigen::Vector3d P0, P1;
        R0 = back_R0 * Tic0_calib_.linear();
        R1 = nav_states_[0].rotation() * Tic0_calib_.linear();
        P0 = back_P0 + back_R0 * Tic0_calib_.translation();
        P1 = nav_states_[0].position() +
             nav_states_[0].rotation() * Tic0_calib_.translation();
        feature_manager_ptr_->removeBackShiftDepth(R0, P0, R1, P1);
      } else {
        feature_manager_ptr_->removeBack();
      }
    }
  } else {
    if (frame_cnt_ == window_size_) {
      time_stamps_[frame_cnt_ - 1] = time_stamps_[frame_cnt_];
      nav_states_[frame_cnt_ - 1] = nav_states_[frame_cnt_];
      if (dataflow_ptr_->useIMU()) {
        imu_bias_[frame_cnt_ - 1] = imu_bias_[frame_cnt_];

        for (size_t i = 0; i < dt_buf_[frame_cnt_].size(); i++) {
          double tmp_dt = dt_buf_[frame_cnt_][i];
          IMUMeasure tmp_imu = imu_meas_buf_[frame_cnt_][i];

          imu_preintegrations_[frame_cnt_ - 1]->push_back(
              tmp_dt, tmp_imu.head<3>(), tmp_imu.tail<3>());

          dt_buf_[frame_cnt_ - 1].push_back(tmp_dt);
          imu_meas_buf_[frame_cnt_ - 1].push_back(tmp_imu);
        }

        imu_preintegrations_[window_size_]->reset(
            imu_meas_0_.head<3>(), imu_meas_0_.tail<3>(),
            imu_bias_[window_size_].head<3>(),
            imu_bias_[window_size_].tail<3>());
        dt_buf_[window_size_].clear();
        imu_meas_buf_[window_size_].clear();
      }

      // Estimator::slideWindowNew()
      // TODO: sum_of_front++;
      feature_manager_ptr_->removeFront(frame_cnt_);
    }
  }
}

void VIOdometry::predictIMU(double t, Eigen::Vector3d linear_acceleration,
                            Eigen::Vector3d angular_velocity) {
  double dt = t - latest_time_;
  latest_time_ = t;

  Eigen::Quaterniond latest_Q = latest_nav_state_.quaternion();
  Eigen::Vector3d latest_P = latest_nav_state_.position();
  Eigen::Vector3d latest_V = latest_nav_state_.velocity();
  Eigen::Vector3d latest_acc_0 = latest_imu_meas_.head<3>();
  Eigen::Vector3d latest_gyr_0 = latest_imu_meas_.tail<3>();
  Eigen::Vector3d latest_Ba = latest_imu_bias_.head<3>();
  Eigen::Vector3d latest_Bg = latest_imu_bias_.tail<3>();

  Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - gravity;
  Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
  latest_Q = latest_Q * Converter::deltaQ(un_gyr * dt);
  latest_nav_state_.setQuaternion(latest_Q);

  Eigen::Vector3d un_acc_1 =
      latest_Q * (linear_acceleration - latest_Ba) - gravity;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
  latest_V = latest_V + dt * un_acc;
  latest_nav_state_.setPosition(latest_P);
  latest_nav_state_.setVelocity(latest_V);

  // latest_acc_0 = linear_acceleration;
  // latest_gyr_0 = angular_velocity;
  latest_imu_meas_.head<3>() = linear_acceleration;
  latest_imu_meas_.tail<3>() = angular_velocity;
}

void VIOdometry::startTracking() {
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

void VIOdometry::trackImage(double t, const cv::Mat &img0,
                            const cv::Mat &img1) {
  track_img_cnt_++;
  PointsTrack frame_features;
  TicToc featureTrackerTime;
  feature_tracker_ptr_->trackImage(t, img0, img1, frame_features);

  // show image track results;
  cv::Mat imgTrack = feature_tracker_ptr_->getTrackImage();
  dataflow_ptr_->pubTrackImage(t, imgTrack);

  if (track_img_cnt_ % 2 == 0) {
    std::unique_lock<std::mutex> lock(buff_mutex_);
    feature_buf_.push(std::make_pair(t, frame_features));
  }
}

void VIOdometry::initIMUPose(std::vector<IMUMeasureTime> &imu_intv) {
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
