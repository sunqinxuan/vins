/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 14:18
#
# Filename: convert_matrix.cpp
#
# Description:
#
************************************************/

#include "tools/convert_matrix.hpp"

namespace vision_localization {
void Converter::NormRotationMatrix(Eigen::Matrix3f &Input) {
  Eigen::Quaternionf qr(Input);
  qr.normalize();
  Input = qr.toRotationMatrix();
}
void Converter::NormTransformMatrix(Eigen::Matrix4f &Input) {
  Eigen::Quaternionf qr(Input.block<3, 3>(0, 0));
  qr.normalize();
  Input.topLeftCorner(3, 3) = qr.toRotationMatrix();
}
void Converter::NormTransformMatrix(Eigen::Isometry3f &Input) {
  Eigen::Quaternionf qr(Input.linear());
  qr.normalize();
  Input.linear() = qr.toRotationMatrix();
}

Eigen::Matrix4f Converter::toMatrixInv4f(const Eigen::Matrix4f &M) {
  Eigen::Matrix3f R = M.topLeftCorner(3, 3);
  Eigen::Vector3f t = M.topRightCorner(3, 1);
  Eigen::Matrix3f R_Inv = R.transpose();
  Eigen::Vector3f t_Inv = -R_Inv * t;
  Eigen::Matrix4f M_Inv = Eigen::Matrix4f::Identity();

  M_Inv.topLeftCorner(3, 3) = R_Inv;
  M_Inv.topRightCorner(3, 1) = t_Inv;

  return M_Inv;
}

Eigen::Matrix3f
Converter::toRotationMatrix(const std::vector<float> &v) // qw,qx,qy,qz
{
  Eigen::Quaternionf Qrotaion(v[0], v[1], v[2], v[3]);
  Qrotaion.normalize();
  return Qrotaion.toRotationMatrix();
}
Eigen::Matrix3f Converter::toRotationMatrix(float real_time,
                                            Eigen::Vector3f angular_rate) {
  // angular_rate_为角速度， angle为一段时间内旋转的角度
  Eigen::Vector3f angle =
      angular_rate * real_time; // angle为相对于当前视觉第一个点所在坐标系的RPY
  // ZYX的旋转顺序，内旋，R表示旋转后坐标系到旋转前坐标系
  Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ()); // yaw
  Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY()); // pitch
  Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX()); // roll
  Eigen::AngleAxisf t_V;
  t_V = t_Vz * t_Vy * t_Vx;
  Eigen::Quaternionf q_V(t_V.matrix());
  // q_V.normalize();
  return q_V.toRotationMatrix();
}

std::vector<float> Converter::toQuaternion(const Eigen::Matrix3f &eigMat) {
  Eigen::Quaternionf q(eigMat);
  q.normalize();

  std::vector<float> v(4);
  v[0] = q.w();
  v[1] = q.x();
  v[2] = q.y();
  v[3] = q.z();
  return v;
}
// roll pitch yaw
Eigen::Vector3f Converter::toEulerAngle(const Eigen::Matrix3f &q) {
  return toEulerAngle(Eigen::Quaternionf(q));
}
/*
 * 输入：R_ab a为旋转前参考系，b为旋转后坐标系
 * 输出：Roll(x),Pitch(y),Yaw(z)
 * 条件：1.右手系;2.旋转顺序为Z->Y->X; 3.旋转方式为内旋(基于上一次旋转的结果);
 */
Eigen::Vector3f Converter::toEulerAngle(const Eigen::Quaternionf &q) {
  Eigen::Vector3f rpy;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  rpy(0) = (float)atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    rpy(1) = (float)copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    rpy(1) = (float)asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  rpy(2) = (float)atan2(siny_cosp, cosy_cosp);

  return rpy;
}
/*
 * 输入：R_ab a为旋转前参考系，b为旋转后坐标系
 * 输出：Roll(x),Pitch(y),Yaw(z)
 * 条件：1.右手系;2.旋转顺序为Z->Y->X; 3.旋转方式为内旋(基于上一次旋转的结果);
 */
Eigen::Vector3d Converter::toEulerAngle(const Eigen::Quaterniond &q) {
  Eigen::Vector3d rpy;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  rpy(0) = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    rpy(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    rpy(1) = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  rpy(2) = atan2(siny_cosp, cosy_cosp);
  return rpy;
}

double Converter::toEulerRollAngle(const Eigen::Matrix3f &q) {
  return toEulerRollAngle(Eigen::Quaternionf(q));
}
double Converter::toEulerRollAngle(const Eigen::Quaternionf &q) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  double roll = atan2(sinr_cosp, cosr_cosp);
  return (float)roll;
}
double Converter::toEulerPitchAngle(const Eigen::Matrix3f &q) {
  return toEulerPitchAngle(Eigen::Quaternionf(q));
}
double Converter::toEulerPitchAngle(const Eigen::Quaternionf &q) {
  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  double pitch;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);
  return (float)pitch;
}
double Converter::toEulerYawAngle(const Eigen::Matrix3f &q) {
  return toEulerYawAngle(Eigen::Quaternionf(q));
}
double Converter::toEulerYawAngle(const Eigen::Quaternionf &q) {
  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  double yaw = atan2(siny_cosp, cosy_cosp);
  return (float)yaw;
}
float Converter::Matrix4ftoYaw(const Eigen::Matrix4f &M) {
  Eigen::Matrix3f R = M.topLeftCorner(3, 3);
  Eigen::Vector3f YPR = R.eulerAngles(2, 1, 0);

  float yaw = YPR(0);
  return yaw;
}

Eigen::Matrix3f Converter::toSkewSym(const Eigen::Vector3f &v) {
  Eigen::Matrix3f mat = Eigen::Matrix3f::Zero();
  mat(0, 1) = -v(2);
  mat(0, 2) = v(1);
  mat(1, 2) = -v(0);
  mat(1, 0) = v(2);
  mat(2, 0) = -v(1);
  mat(2, 1) = v(0);
  return mat;
}
Eigen::Vector3f Converter::toSkewVec(const Eigen::Matrix3f &m) {
  return Eigen::Vector3f(m(2, 1), m(0, 2), m(1, 0));
}
Eigen::Matrix3f Converter::Exp(const Eigen::Vector3f &v) {
  float phy = v.norm();
  Eigen::Vector3f u = v.normalized();
  return Eigen::Matrix3f::Identity() + sin(phy) * toSkewSym(u) +
         (1 - cos(phy)) * toSkewSym(u) * toSkewSym(u);
}
Eigen::Vector3f Converter::Log(const Eigen::Matrix3f &m) {
  float phi = acos(0.5 * (m.trace() - 1.0));
  if (fabs(phi) < 1e-4)
    return Eigen::Vector3f::Zero();
  else {
    return phi * toSkewVec(m - m.transpose()) / (2.0 * sin(phi));
  }
}
} // namespace vision_localization
