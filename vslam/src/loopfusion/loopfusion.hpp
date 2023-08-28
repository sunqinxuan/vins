/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 15:03
#
# Filename:		loopfusion.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_LOOPFUSION_HPP_
#define VSLAM_LOOPFUSION_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "dataflow/loopdataflow.hpp"
//#include "optimizer/factor_graph_optimizer.hpp"
//#include "tools/converter.hpp"
//#include "tools/geometry.hpp"
#include "loopfusion/keyframe.hpp"
#include "loopfusion/pose_graph.hpp"
#include "tools/message_print.hpp"
//#include "tools/type_redefine.hpp"

namespace vslam {

class LoopFusion {
public:
  LoopFusion(ros::NodeHandle &nh, std::string config_file_path);

  void run();
  void clearState();

private:
  // void startTracking();
  // void trackImage(double t, const cv::Mat &img0, const cv::Mat &img1);

private:
  std::thread thread_track_;
  std::mutex buff_mutex_;
  std::mutex prog_mutex_;

  std::shared_ptr<LoopDataFlow> dataflow_ptr_;
  std::shared_ptr<PoseGraph> pose_graph_ptr_;
  std::shared_ptr<KeyFrame> key_frame_ptr_;

  int frame_idx_ = 0;
  int sequence_ = 1;
};
} // namespace vslam

#endif
