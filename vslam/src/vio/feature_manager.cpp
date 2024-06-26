/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "vio/feature_manager.hpp"

namespace vslam {

int FeaturePerId::endFrame() {
  return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager() {
  focal_length_ = 460.0;
  min_parallax_ = 10.0 / focal_length_;
  window_size_ = 10;
}

FeatureManager::FeatureManager(const double focal_length,
                               const double min_parallax,
                               const int window_size) {
  focal_length_ = focal_length;
  min_parallax_ = min_parallax;
  window_size_ = window_size;
}

void FeatureManager::clearState() { feature.clear(); }

bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const PointsTrack &image,
                                             double td) {
  DEBUG("input feature: ", (int)image.size());
  DEBUG("num of feature: ", getFeatureCount());
  double parallax_sum = 0;
  int parallax_num = 0;
  last_track_num = 0;
  last_average_parallax = 0;
  new_feature_num = 0;
  long_track_num = 0;
  for (auto &id_pts : image) {
    FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
    assert(id_pts.second[0].first == 0);
    if (id_pts.second.size() == 2) {
      f_per_fra.rightObservation(id_pts.second[1].second);
      assert(id_pts.second[1].first == 1);
    }

    int feature_id = id_pts.first;
    auto it = find_if(feature.begin(), feature.end(),
                      [feature_id](const FeaturePerId &it) {
                        return it.feature_id == feature_id;
                      });

    if (it == feature.end()) {
      feature.push_back(FeaturePerId(feature_id, frame_count));
      feature.back().feature_per_frame.push_back(f_per_fra);
      new_feature_num++;
    } else if (it->feature_id == feature_id) {
      it->feature_per_frame.push_back(f_per_fra);
      last_track_num++;
      if (it->feature_per_frame.size() >= 4)
        long_track_num++;
    }
  }

  // if (frame_count < 2 || last_track_num < 20)
  // if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 *
  // last_track_num)
  if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 ||
      new_feature_num > 0.5 * last_track_num)
    return true;

  for (auto &it_per_id : feature) {
    if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >=
            frame_count - 1) {
      parallax_sum += compensatedParallax2(it_per_id, frame_count);
      parallax_num++;
    }
  }

  if (parallax_num == 0) {
    return true;
  } else {
    DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
    DEBUG("current parallax: %lf", parallax_sum / parallax_num * focal_length_);
    last_average_parallax = parallax_sum / parallax_num * focal_length_;
    return parallax_sum / parallax_num >= min_parallax_;
  }
}

int FeatureManager::getFeatureCount() {
  int cnt = 0;
  for (auto &it : feature) {
    it.used_num = it.feature_per_frame.size();
    if (it.used_num >= 4) {
      cnt++;
    }
  }
  return cnt;
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id,
                                            int frame_count) {
  // check the second last frame is keyframe or not
  // parallax betwwen seconde last frame and third last frame
  const FeaturePerFrame &frame_i =
      it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
  const FeaturePerFrame &frame_j =
      it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

  double ans = 0;
  Eigen::Vector3d p_j = frame_j.point;

  double u_j = p_j(0);
  double v_j = p_j(1);

  Eigen::Vector3d p_i = frame_i.point;
  Eigen::Vector3d p_i_comp;

  // int r_i = frame_count - 2;
  // int r_j = frame_count - 1;
  // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] *
  // ric[camera_id_i] * p_i;
  p_i_comp = p_i;
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;
  double du = u_i - u_j, dv = v_i - v_j;

  double dep_i_comp = p_i_comp(2);
  double u_i_comp = p_i_comp(0) / dep_i_comp;
  double v_i_comp = p_i_comp(1) / dep_i_comp;
  double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

  ans = std::max(ans, sqrt(std::min(du * du + dv * dv,
                                    du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}

//(int frameCnt, Eigen::Vector3d Ps[], Eigen::Matrix3d Rs[], Eigen::Vector3d
// tic[], Eigen::Matrix3d ric[])
void FeatureManager::initFramePoseByPnP(int frameCnt,
                                        std::vector<NavState> &navStates,
                                        const Eigen::Isometry3d &Tic) {
  if (frameCnt > 0) {
    std::vector<cv::Point2f> pts2D;
    std::vector<cv::Point3f> pts3D;
    DEBUG("feature size: ", feature.size());
    for (auto &it_per_id : feature) {
      //			DEBUG("\t",it_per_id.estimated_depth);
      if (it_per_id.estimated_depth > 0) {
        int index = frameCnt - it_per_id.start_frame;
        if ((int)it_per_id.feature_per_frame.size() >= index + 1) {
          // Eigen::Vector3d ptsInCam = ric[0] *
          // (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth)
          // + tic[0];
          Eigen::Vector3d ptsInCam =
              Tic.linear() * (it_per_id.feature_per_frame[0].point *
                              it_per_id.estimated_depth) +
              Tic.translation();
          // Eigen::Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam +
          // Ps[it_per_id.start_frame];
          Eigen::Vector3d ptsInWorld =
              navStates[it_per_id.start_frame].rotation() * ptsInCam +
              navStates[it_per_id.start_frame].position();

          cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
          cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(),
                              it_per_id.feature_per_frame[index].point.y());
          pts3D.push_back(point3d);
          pts2D.push_back(point2d);
          // std::cout << "\t" << point2d.x << ", " << point2d.y << std::endl;
        }
      }
    }
    Eigen::Matrix3d RCam;
    Eigen::Vector3d PCam;
    // trans to w_T_cam
    // RCam = Rs[frameCnt - 1] * ric[0];
    // PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];
    RCam = navStates[frameCnt - 1].rotation() * Tic.linear();
    PCam = navStates[frameCnt - 1].rotation() * Tic.translation() +
           navStates[frameCnt - 1].position();

    DEBUG("RCam");
    DEBUG(RCam);
    DEBUG("PCam:", PCam.transpose());
    DEBUG("pts2D:", pts2D.size());
    DEBUG("pts3D:", pts3D.size());
    if (solvePoseByPnP(RCam, PCam, pts2D, pts3D)) {
      // trans to w_T_imu
      // Rs[frameCnt] = RCam * ric[0].transpose();
      // Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;
      navStates[frameCnt].setRotation(RCam * Tic.linear().transpose());
      navStates[frameCnt].setPosition(
          -RCam * Tic.linear().transpose() * Tic.translation() + PCam);

      Eigen::Quaterniond Q(navStates[frameCnt].rotation());
      std::cout << "frameCnt: " << frameCnt << " pnp Q " << Q.w() << " "
                << Q.vec().transpose() << std::endl;
      std::cout << "frameCnt: " << frameCnt << " pnp P "
                << navStates[frameCnt].position().transpose() << std::endl;
    }
  }
}

bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                    std::vector<cv::Point2f> &pts2D,
                                    std::vector<cv::Point3f> &pts3D) {
  Eigen::Matrix3d R_initial;
  Eigen::Vector3d P_initial;

  // w_T_cam ---> cam_T_w
  R_initial = R.inverse();
  P_initial = -(R_initial * P);

  printf("pnp size %d \n", (int)pts2D.size());
  if (int(pts2D.size()) < 4) {
    printf("feature tracking not enough, please slowly move you device!  \n");
    return false;
  }
  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(R_initial, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(P_initial, t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  bool pnp_succ;
  pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
  // pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 /
  // focalLength, 0.99, inliers);

  if (!pnp_succ) {
    printf("pnp failed ! \n");
    return false;
  }
  cv::Rodrigues(rvec, r);
  // cout << "r " << endl << r << endl;
  Eigen::MatrixXd R_pnp;
  cv::cv2eigen(r, R_pnp);
  Eigen::MatrixXd T_pnp;
  cv::cv2eigen(t, T_pnp);

  // cam_T_w ---> w_T_cam
  R = R_pnp.transpose();
  P = R * (-T_pnp);

  return true;
}

//(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
void FeatureManager::triangulate(std::vector<NavState> &navStates,
                                 const Eigen::Isometry3d &Tic0,
                                 const Eigen::Isometry3d &Tic1) {
  for (auto &it_per_id : feature) {
    // DEBUG(it_per_id.feature_id, "\t", it_per_id.estimated_depth);
    if (it_per_id.estimated_depth > 0)
      continue;

    if (it_per_id.feature_per_frame[0].is_stereo) {
      int imu_i = it_per_id.start_frame;
      // DEBUG("\tis_stereo ", imu_i);
      Eigen::Matrix<double, 3, 4> leftPose;
      // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
      // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
      Eigen::Vector3d t0 = navStates[imu_i].position() +
                           navStates[imu_i].rotation() * Tic0.translation();
      Eigen::Matrix3d R0 = navStates[imu_i].rotation() * Tic0.linear();
      leftPose.leftCols<3>() = R0.transpose();
      leftPose.rightCols<1>() = -R0.transpose() * t0;
      // std::cout << "\tleft pose " << leftPose << std::endl;

      Eigen::Matrix<double, 3, 4> rightPose;
      // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
      // Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
      Eigen::Vector3d t1 = navStates[imu_i].position() +
                           navStates[imu_i].rotation() * Tic1.translation();
      Eigen::Matrix3d R1 = navStates[imu_i].rotation() * Tic1.linear();
      rightPose.leftCols<3>() = R1.transpose();
      rightPose.rightCols<1>() = -R1.transpose() * t1;
      // std::cout << "\tright pose " << rightPose << std::endl;

      Eigen::Vector2d point0, point1;
      Eigen::Vector3d point3d;
      point0 = it_per_id.feature_per_frame[0].point.head(2);
      point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
      // std::cout << "\tpoint0 " << point0.transpose() << std::endl;
      // std::cout << "\tpoint1 " << point1.transpose() << std::endl;

      triangulatePoint(leftPose, rightPose, point0, point1, point3d);
      Eigen::Vector3d localPoint;
      localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
      // std::cout << "\tlocalPoint " << localPoint.transpose() << std::endl;
      double depth = localPoint.z();
      if (depth > 0)
        it_per_id.estimated_depth = depth;
      else
        it_per_id.estimated_depth = 5.0; // INIT_DEPTH;

      // Eigen::Vector3d ptsGt = pts_gt[it_per_id.feature_id];
      // printf("stereo %d pts: %f %f %f gt: %f %f %f \n", it_per_id.feature_id,
      //       point3d.x(), point3d.y(), point3d.z(), ptsGt.x(), ptsGt.y(),
      //       ptsGt.z());

      continue;

    } else if (it_per_id.feature_per_frame.size() > 1) {
      int imu_i = it_per_id.start_frame;
      // DEBUG("\tnot_stereo ", imu_i);
      Eigen::Matrix<double, 3, 4> leftPose;
      // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
      // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
      Eigen::Vector3d t0 = navStates[imu_i].position() +
                           navStates[imu_i].rotation() * Tic0.translation();
      Eigen::Matrix3d R0 = navStates[imu_i].rotation() * Tic0.linear();
      leftPose.leftCols<3>() = R0.transpose();
      leftPose.rightCols<1>() = -R0.transpose() * t0;

      imu_i++;
      Eigen::Matrix<double, 3, 4> rightPose;
      // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
      // Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
      Eigen::Vector3d t1 = navStates[imu_i].position() +
                           navStates[imu_i].rotation() * Tic0.translation();
      Eigen::Matrix3d R1 = navStates[imu_i].rotation() * Tic0.linear();
      rightPose.leftCols<3>() = R1.transpose();
      rightPose.rightCols<1>() = -R1.transpose() * t1;

      Eigen::Vector2d point0, point1;
      Eigen::Vector3d point3d;
      point0 = it_per_id.feature_per_frame[0].point.head(2);
      point1 = it_per_id.feature_per_frame[1].point.head(2);
      triangulatePoint(leftPose, rightPose, point0, point1, point3d);
      Eigen::Vector3d localPoint;
      localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
      // std::cout << "\tlocalPoint " << localPoint.transpose() << std::endl;
      double depth = localPoint.z();
      if (depth > 0)
        it_per_id.estimated_depth = depth;
      else
        it_per_id.estimated_depth = 5.0; // INIT_DEPTH;

      // Vector3d ptsGt = pts_gt[it_per_id.feature_id];
      // printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id,
      // point3d.x(), point3d.y(), point3d.z(),
      // ptsGt.x(), ptsGt.y(), ptsGt.z());
      continue;
    }
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
    int svd_idx = 0;

    Eigen::Matrix<double, 3, 4> P0;
    // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
    // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
    Eigen::Vector3d t0 = navStates[imu_i].position() +
                         navStates[imu_i].rotation() * Tic0.translation();
    Eigen::Matrix3d R0 = navStates[imu_i].rotation() * Tic0.linear();
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;

      // Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
      // Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
      Eigen::Vector3d t1 = navStates[imu_j].position() +
                           navStates[imu_j].rotation() * Tic0.translation();
      Eigen::Matrix3d R1 = navStates[imu_j].rotation() * Tic0.linear();
      Eigen::Vector3d t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d R = R0.transpose() * R1;
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = R.transpose();
      P.rightCols<1>() = -R.transpose() * t;
      Eigen::Vector3d f = it_per_frame.point.normalized();
      svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
      svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

      if (imu_i == imu_j)
        continue;
    }
    // ROS_ASSERT(svd_idx == svd_A.rows());
    Eigen::Vector4d svd_V =
        Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV)
            .matrixV()
            .rightCols<1>();
    // std::cout << "\tsvd_V:" << svd_V.transpose() << std::endl;
    double svd_method = svd_V[2] / svd_V[3];
    // it_per_id->estimated_depth = -b / A;
    // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

    it_per_id.estimated_depth = svd_method;
    // it_per_id->estimated_depth = INIT_DEPTH;

    if (it_per_id.estimated_depth < 0.1) {
      it_per_id.estimated_depth = 5.0; // INIT_DEPTH;
    }
  }
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                                      Eigen::Matrix<double, 3, 4> &Pose1,
                                      Eigen::Vector2d &point0,
                                      Eigen::Vector2d &point1,
                                      Eigen::Vector3d &point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

Eigen::VectorXd FeatureManager::getDepthVector() {
  Eigen::VectorXd dep_vec(getFeatureCount());
  int feature_index = -1;
  for (auto &it_per_id : feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;
    dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    // dep_vec(++feature_index) = it_per_id->estimated_depth;
  }
  return dep_vec;
}

void FeatureManager::setDepth(const Eigen::VectorXd &x) {
  int feature_index = -1;
  for (auto &it_per_id : feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4)
      continue;

    it_per_id.estimated_depth = 1.0 / x(++feature_index);
    // INFO("feature id %d , start_frame %d, depth %f ", it_per_id.feature_id,
    //     it_per_id.start_frame, it_per_id.estimated_depth);
    if (it_per_id.estimated_depth < 0) {
      it_per_id.solve_flag = 2;
    } else
      it_per_id.solve_flag = 1;
  }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R,
                                          Eigen::Vector3d marg_P,
                                          Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P) {
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    if (it->start_frame != 0)
      it->start_frame--;
    else {
      Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
      it->feature_per_frame.erase(it->feature_per_frame.begin());
      if (it->feature_per_frame.size() < 2) {
        feature.erase(it);
        continue;
      } else {
        Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
        Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
        Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
        double dep_j = pts_j(2);
        if (dep_j > 0)
          it->estimated_depth = dep_j;
        else
          it->estimated_depth = 5.0; // INIT_DEPTH;
      }
    }
    // remove tracking-lost feature after marginalize
    // if (it->endFrame() < WINDOW_SIZE - 1)
    //{
    //    feature.erase(it);
    //}
  }
}

void FeatureManager::removeBack() {
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    if (it->start_frame != 0)
      it->start_frame--;
    else {
      it->feature_per_frame.erase(it->feature_per_frame.begin());
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

void FeatureManager::removeFront(int frame_count) {
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    if (it->start_frame == frame_count) {
      it->start_frame--;
    } else {
      int j = window_size_ - 1 - it->start_frame;
      if (it->endFrame() < frame_count - 1)
        continue;
      it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

void FeatureManager::removeOutlier(std::set<int> &outlierIndex) {
  std::set<int>::iterator itSet;
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;
    int index = it->feature_id;
    itSet = outlierIndex.find(index);
    if (itSet != outlierIndex.end()) {
      feature.erase(it);
      // printf("remove outlier %d \n", index);
    }
  }
}

void FeatureManager::removeFailures() {
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;
    if (it->solve_flag == 2)
      feature.erase(it);
  }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
  for (auto &it : feature) {
    if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
      Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
      int idx_l = frame_count_l - it.start_frame;
      int idx_r = frame_count_r - it.start_frame;

      a = it.feature_per_frame[idx_l].point;

      b = it.feature_per_frame[idx_r].point;

      corres.push_back(std::make_pair(a, b));
    }
  }
  return corres;
}

// FeatureManager::FeatureManager(Matrix3d _Rs[])
//    : Rs(_Rs)
//{
//    for (int i = 0; i < NUM_OF_CAM; i++)
//        ric[i].setIdentity();
//}

/*
void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}












void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}



















*/
} // namespace vslam
