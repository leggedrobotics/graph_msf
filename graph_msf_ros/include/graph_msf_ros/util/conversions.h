/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H
#define MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H

#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

namespace graph_msf {

inline void odomMsgToEigen(const nav_msgs::Odometry& odomLidar, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  Eigen::Vector3d t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

inline void odomMsgToTf(const nav_msgs::Odometry& odomLidar, tf::Transform& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  tf::Vector3 tf_t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  T = tf::Transform(tf_q, tf_t);
}

inline tf::Transform matrix3ToTf(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  tf::Transform tf_R;
  tf_R.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_R.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  return tf_R;
}

inline tf::Transform matrix4ToTf(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

inline void tfToMatrix4(const tf::Transform& tf_T, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q = tf_T.getRotation();
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  Eigen::Vector3d t(tf_T.getOrigin().getX(), tf_T.getOrigin().getY(), tf_T.getOrigin().getZ());
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

inline tf::Transform pose3ToTf(const Eigen::Matrix3d& T) {
  Eigen::Quaterniond q(T);
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

}  // namespace graph_msf

#endif  // MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H
