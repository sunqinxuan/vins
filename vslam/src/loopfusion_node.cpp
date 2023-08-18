/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 15:00
#
# Filename:		loopfusion_node.cpp
#
# Description:
#
************************************************/

#include "loopfusion/loopfusion.hpp"

using namespace vslam;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "loopfusion_node");
  ros::NodeHandle nh;

  std::string work_space_path;
  nh.param<std::string>("work_space_path", work_space_path, "");

  std::string config_file_path = work_space_path + "/config/euroc/";
  // std::string config_file_path = work_space_path + "/config/kitti/";
  std::shared_ptr<LoopFusion> loop_fusion_ptr =
      std::make_shared<LoopFusion>(nh, config_file_path);

  loop_fusion_ptr->run();
  ros::spin();

  return 0;
}
