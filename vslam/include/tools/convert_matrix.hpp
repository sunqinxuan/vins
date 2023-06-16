/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 14:20
#
# Filename: convert_matrix.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_TOOLS_CONVERT_MATRIX_HPP_
#define VSLAM_TOOLS_CONVERT_MATRIX_HPP_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace vision_localization {
class Converter {
public:
  static void NormRotationMatrix(Eigen::Matrix3f &Input);
  static void NormTransformMatrix(Eigen::Matrix4f &Input);
  static void NormTransformMatrix(Eigen::Isometry3f &Input);

  static Eigen::Matrix4f toMatrixInv4f(const Eigen::Matrix4f &Input);
  static Eigen::Matrix3f
  toRotationMatrix(const std::vector<float> &v); // qw,qx,qy,qz
  static Eigen::Matrix3f toRotationMatrix(float real_time,
                                          Eigen::Vector3f angular_rate);
  static std::vector<float> toQuaternion(const Eigen::Matrix3f &eigMat);

  static Eigen::Vector3f toEulerAngle(const Eigen::Quaternionf &q);
  static Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond &q);
  static double toEulerRollAngle(const Eigen::Quaternionf &q);
  static double toEulerPitchAngle(const Eigen::Quaternionf &q);
  static double toEulerYawAngle(const Eigen::Quaternionf &q);

  static Eigen::Vector3f toEulerAngle(const Eigen::Matrix3f &q);
  static double toEulerRollAngle(const Eigen::Matrix3f &q);
  static double toEulerPitchAngle(const Eigen::Matrix3f &q);
  static double toEulerYawAngle(const Eigen::Matrix3f &q);
  static float Matrix4ftoYaw(const Eigen::Matrix4f &M);

  static Eigen::Matrix3f toSkewSym(const Eigen::Vector3f &v);
  static Eigen::Vector3f toSkewVec(const Eigen::Matrix3f &m);
  static Eigen::Matrix3f Exp(const Eigen::Vector3f &v);
  static Eigen::Vector3f Log(const Eigen::Matrix3f &m);
};
} // namespace vision_localization
#endif
