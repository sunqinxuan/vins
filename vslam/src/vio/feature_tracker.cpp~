/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "frontend/feature_tracker.hpp"

namespace vslam {

FeatureTracker::FeatureTracker(int max_cnt, int min_dist, double F_thres)
    : MAX_CNT(max_cnt), MIN_DIST(min_dist), F_THRESHOLD(F_thres) {
  // stereo_cam = 0;
  n_id = 0;
  hasPrediction = false;
}

void FeatureTracker::readCameraIntrinsics(const std::string &cam0_file,
                                          const std::string &cam1_file) {
  // for (size_t i = 0; i < calib_file.size(); i++)
  //{
  // INFO("reading paramerter of camera %s", cam0_file.c_str());
  camera0 = CameraFactory::instance()->generateCameraFromYamlFile(cam0_file);
  // INFO("reading paramerter of camera %s", cam1_file.c_str());
  camera1 = CameraFactory::instance()->generateCameraFromYamlFile(cam1_file);
  // m_camera.push_back(camera);
  //}
  // if (calib_file.size() == 2)
  // stereo_cam = 1;
  // std::string c0 = camera0->parametersToString();
  // std::string c1 = camera1->parametersToString();
  // std::cout << "cam0: " << std::endl<< c0 << std::endl;
  // std::cout << "cam1: " << std::endl<< c1 << std::endl;
}

bool FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img,
                                const cv::Mat &_img1,
                                PointsTrack &featureFrame) {
  TicToc t_r;
  cur_time = _cur_time;
  cur_img = _img;
  row = cur_img.rows;
  col = cur_img.cols;
  cv::Mat rightImg = _img1;
  //{
  //    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  //    clahe->apply(cur_img, cur_img);
  //    if(!rightImg.empty())
  //        clahe->apply(rightImg, rightImg);
  //}
  cur_pts.clear();

  if (prev_pts.size() > 0) {
    TicToc t_o;
    std::vector<uchar> status;
    std::vector<float> err;
    if (hasPrediction) {
      cur_pts = predict_pts;
      cv::calcOpticalFlowPyrLK(
          prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21),
          1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);

      int succ_num = 0;
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i])
          succ_num++;
      }
      if (succ_num < 10)
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                                 err, cv::Size(21, 21), 3);
    } else
      cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                               err, cv::Size(21, 21), 3);
    // reverse check
    // TODO set optional
    // if (FLOW_BACK)
    {
      std::vector<uchar> reverse_status;
      std::vector<cv::Point2f> reverse_pts = prev_pts;
      cv::calcOpticalFlowPyrLK(
          cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err,
          cv::Size(21, 21), 1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
      // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts,
      // reverse_status, err, cv::Size(21, 21), 3);
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] &&
            distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
          status[i] = 1;
        } else
          status[i] = 0;
      }
    }

    for (int i = 0; i < int(cur_pts.size()); i++)
      if (status[i] && !inBorder(cur_pts[i]))
        status[i] = 0;
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    // INFO("temporal optical flow costs: ", t_o.toc(),"ms");
    // printf("track cnt %d\n", (int)ids.size());
  }

  for (auto &n : track_cnt)
    n++;

  if (1) {
    // rejectWithF();
    // INFO("set mask begins");
    TicToc t_m;
    setMask();
    // INFO("set mask costs ", t_m.toc(),"ms");

    // INFO("detect feature begins");
    TicToc t_t;
    int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
    if (n_max_cnt > 0) {
      if (mask.empty())
        std::cout << "mask is empty " << std::endl;
      if (mask.type() != CV_8UC1)
        std::cout << "mask type wrong " << std::endl;
      cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01,
                              MIN_DIST, mask);
    } else
      n_pts.clear();
    // INFO("detect feature costs ", t_t.toc(),"ms");

    for (auto &p : n_pts) {
      cur_pts.push_back(p);
      ids.push_back(n_id++);
      track_cnt.push_back(1);
    }
    // printf("feature cnt after add %d\n", (int)ids.size());
  }

  // std::string c0 = camera0->parametersToString();
  // std::cout << "camera0: " << std::endl<< c0 << std::endl;

  cur_un_pts = undistortedPts(cur_pts, camera0);
  pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

  // if (!_img1.empty())// && stereo_cam)
  {
    ids_right.clear();
    cur_right_pts.clear();
    cur_un_right_pts.clear();
    right_pts_velocity.clear();
    cur_un_right_pts_map.clear();
    if (!cur_pts.empty()) {
      // printf("stereo image; track feature on right image\n");
      std::vector<cv::Point2f> reverseLeftPts;
      std::vector<uchar> status, statusRightLeft;
      std::vector<float> err;
      // cur left ---- cur right
      cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts,
                               status, err, cv::Size(21, 21), 3);
      // reverse check cur right ---- cur left
      // TODO set optional
      // if (FLOW_BACK)
      {
        cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts,
                                 reverseLeftPts, statusRightLeft, err,
                                 cv::Size(21, 21), 3);
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
              distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
            status[i] = 1;
          else
            status[i] = 0;
        }
      }

      ids_right = ids;
      reduceVector(cur_right_pts, status);
      reduceVector(ids_right, status);
      // only keep left-right pts

      // reduceVector(cur_pts, status);
      // reduceVector(ids, status);
      // reduceVector(track_cnt, status);
      // reduceVector(cur_un_pts, status);
      // reduceVector(pts_velocity, status);

      cur_un_right_pts = undistortedPts(cur_right_pts, camera1);
      right_pts_velocity =
          ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map,
                      prev_un_right_pts_map);
    }
    prev_un_right_pts_map = cur_un_right_pts_map;
  }
  // TODO set optional
  // if (SHOW_TRACK)
  drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  prev_un_pts_map = cur_un_pts_map;
  prev_time = cur_time;
  hasPrediction = false;

  prevLeftPtsMap.clear();
  for (size_t i = 0; i < cur_pts.size(); i++)
    prevLeftPtsMap[ids[i]] = cur_pts[i];

  // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  featureFrame.clear();
  for (size_t i = 0; i < ids.size(); i++) {
    int feature_id = ids[i];
    double x, y, z;
    x = cur_un_pts[i].x;
    y = cur_un_pts[i].y;
    z = 1;
    double p_u, p_v;
    p_u = cur_pts[i].x;
    p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x, velocity_y;
    velocity_x = pts_velocity[i].x;
    velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }

  // if (!_img1.empty())// && stereo_cam)
  {
    for (size_t i = 0; i < ids_right.size(); i++) {
      int feature_id = ids_right[i];
      double x, y, z;
      x = cur_un_right_pts[i].x;
      y = cur_un_right_pts[i].y;
      z = 1;
      double p_u, p_v;
      p_u = cur_right_pts[i].x;
      p_v = cur_right_pts[i].y;
      int camera_id = 1;
      double velocity_x, velocity_y;
      velocity_x = right_pts_velocity[i].x;
      velocity_y = right_pts_velocity[i].y;

      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
      featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
  }

  // printf("feature track whole time %f\n", t_r.toc());
  return true;
}

void FeatureTracker::setMask() {
  mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < cur_pts.size(); i++)
    cnt_pts_id.push_back(
        std::make_pair(track_cnt[i], std::make_pair(cur_pts[i], ids[i])));

  std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
            [](const std::pair<int, std::pair<cv::Point2f, int>> &a,
               const std::pair<int, std::pair<cv::Point2f, int>> &b) {
              return a.first > b.first;
            });

  cur_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

std::vector<cv::Point2f>
FeatureTracker::ptsVelocity(std::vector<int> &ids,
                            std::vector<cv::Point2f> &pts,
                            std::map<int, cv::Point2f> &cur_id_pts,
                            std::map<int, cv::Point2f> &prev_id_pts) {
  std::vector<cv::Point2f> pts_velocity;
  cur_id_pts.clear();
  for (unsigned int i = 0; i < ids.size(); i++) {
    cur_id_pts.insert(std::make_pair(ids[i], pts[i]));
  }

  // caculate points velocity
  if (!prev_id_pts.empty()) {
    double dt = cur_time - prev_time;

    for (unsigned int i = 0; i < pts.size(); i++) {
      std::map<int, cv::Point2f>::iterator it;
      it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.push_back(cv::Point2f(v_x, v_y));
      } else
        pts_velocity.push_back(cv::Point2f(0, 0));
    }
  } else {
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  return pts_velocity;
}

std::vector<cv::Point2f>
FeatureTracker::undistortedPts(std::vector<cv::Point2f> &pts,
                               camodocal::CameraPtr cam) {
  std::vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    //		std::cout<<"a: "<<a.transpose()<<std::endl;
    // std::string c0 = cam->parametersToString();
    // std::cout << "cam: " << std::endl<< c0 << std::endl;
    cam->liftProjective(a, b);
    //		std::cout<<"b: "<<b.transpose()<<std::endl;
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

bool FeatureTracker::inBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               std::vector<int> &curLeftIds,
                               std::vector<cv::Point2f> &curLeftPts,
                               std::vector<cv::Point2f> &curRightPts,
                               std::map<int, cv::Point2f> &prevLeftPtsMap) {
  // int rows = imLeft.rows;
  int cols = imLeft.cols;
  // if (!imRight.empty() && stereo_cam)
  cv::hconcat(imLeft, imRight, imTrack);
  // else
  //  imTrack = imLeft.clone();
  cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

  for (size_t j = 0; j < curLeftPts.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
    cv::circle(imTrack, curLeftPts[j], 2,
               cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  // if (!imRight.empty() && stereo_cam)
  {
    for (size_t i = 0; i < curRightPts.size(); i++) {
      cv::Point2f rightPt = curRightPts[i];
      rightPt.x += cols;
      cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
      // cv::Point2f leftPt = curLeftPtsTrackRight[i];
      // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
  }

  std::map<int, cv::Point2f>::iterator mapIt;
  for (size_t i = 0; i < curLeftIds.size(); i++) {
    int id = curLeftIds[i];
    mapIt = prevLeftPtsMap.find(id);
    if (mapIt != prevLeftPtsMap.end()) {
      cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second,
                      cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
  }

  // draw prediction
  //
  // for(size_t i = 0; i < predict_pts_debug.size(); i++)
  //{
  //    cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255),
  //    2);
  //}

  // printf("predict pts size %d \n", (int)predict_pts_debug.size());

  // cv::Mat imCur2Compress;
  // cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

cv::Mat FeatureTracker::getTrackImage() { return imTrack; }

/*


double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}











void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()),
un_prev_pts(prev_pts.size()); for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            camera0->liftProjective(Eigen::Vector2d(cur_pts[i].x,
cur_pts[i].y), tmp_p); tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col
/ 2.0; tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            camera0->liftProjective(Eigen::Vector2d(prev_pts[i].x,
prev_pts[i].y), tmp_p); tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col
/ 2.0; tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC,
F_THRESHOLD, 0.99, status); int size_a = cur_pts.size(); reduceVector(prev_pts,
status); reduceVector(cur_pts, status); reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 *
cur_pts.size() / size_a); ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}



void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            camera0->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() /
b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row +
600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0,
0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x,
pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}








void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size
%d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size()); int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            camera0->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}



*/
} // namespace vslam
