/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2023-06-15 10:39
#
# Filename:		slam_node.cpp
#
# Description:
#
************************************************/

#include "frontend/frontend.hpp"

using namespace vslam;
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh;

  // std::string pc_ins_topic;
  bool map_init_from_file;
  std::string work_space_path;
  // nh.param<std::string>("pc_ins_topic", pc_ins_topic, "/synced_pc_ins"); //
  // test
  nh.param<bool>("map_init_from_file", map_init_from_file, true);
  nh.param<std::string>("work_space_path", work_space_path, "");

  // std::cout << "map_init_from_file: " << map_init_from_file << std::endl;
  // std::cout << "work_space_path: " << work_space_path << std::endl;

  std::shared_ptr<FrontEnd> frontend_ptr =
      std::make_shared<FrontEnd>(nh, work_space_path);
	frontend_ptr->clearState();

  // ros::Rate rate(1000);
  // while (ros::ok()) {
  //  ros::spinOnce();
  //  // static TicToc t(0.005);
  //  dataflow_ptr->Run();
  //  // double time = t.ave_toc() * 1000;
  //  // if (time != 0.0) REMIND("[data_pretreat_node] run time ", time, " ms");
  //  rate.sleep();
  //}

  // std::thread sync_thread{sync_process};
  frontend_ptr->run();
  ros::spin();

  return 0;
}
