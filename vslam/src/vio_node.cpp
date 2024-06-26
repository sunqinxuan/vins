/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-08-14 14:37
#
# Filename:		vio_node.cpp
#
# Description:
#
************************************************/

#include "vio/vio.hpp"

using namespace vslam;
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vio_node");
  ros::NodeHandle nh;

  std::string work_space_path;
  nh.param<std::string>("work_space_path", work_space_path, "");

  std::string config_file_path = work_space_path + "/config/euroc/";
  std::shared_ptr<VIOdometry> vio_ptr =
      std::make_shared<VIOdometry>(nh, config_file_path);
  vio_ptr->clearState();

  vio_ptr->run();
  ros::spin();

  return 0;
}
