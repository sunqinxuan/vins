/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-07-03 10:48
#
# Filename: type_redefine.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_TYPE_REDIFINE_
#define VSLAM_TYPE_REDIFINE_

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <map>
#include <string>
#include <vector>

namespace vslam {
// map<feature id, vector<camera id, [x,y,z,u,v,vx,vy]>>
using PointsTrack =
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;
using IMUMeasure = Eigen::Matrix<double, 6, 1>; // [a,w]
using IMUMeasureTime = std::pair<double, IMUMeasure>;
using IMUBias = Eigen::Matrix<double, 6, 1>; // [ba,bw]

} // namespace vslam
#endif
