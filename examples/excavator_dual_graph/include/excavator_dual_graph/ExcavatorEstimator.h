/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef EXCAVATORESTIMATOR_H
#define EXCAVATORESTIMATOR_H

// std
#include <chrono>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Workspace
#include "excavator_dual_graph/ExcavatorStaticTransforms.h"
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace excavator_se {

class ExcavatorEstimator : public graph_msf::GraphMsfRos {
 public:
  ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);

 private:
  // Publish State
  void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik, const Eigen::Vector3d& Ic_v_W_Ic,
                     const Eigen::Vector3d& I_w_W_I) override;

  // Parameter Reading Commodity Function
  void readParams_(const ros::NodeHandle& privateNode);

  void initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) override;

  void initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) override;

  void initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr);

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lidarRate_ = 5.0;
  double gnssLeftRate_ = 20.0;
  double gnssRightRate_ = 20.0;

  // Noise
  Eigen::Matrix<double, 6, 1> poseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> poseUnaryNoise_;
  double gnssPositionUnaryNoise_ = 1.0;  // in [m]
  double gnssHeadingUnaryNoise_ = 1.0;   // in [rad]

  /// Flags
  bool usingLioFlag_ = true;

  // ROS Related stuff ----------------------------

  // Callbacks
  void imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);

  // Node
  ros::NodeHandle privateNode_;

  // Subscribers
  // Instances
  ros::Subscriber subImu_;
  ros::Subscriber subLidarOdometry_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssLeft_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssRight_;
  tf::TransformListener tfListener_;
  // GNSS Sync Policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> GnssExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<GnssExactSyncPolicy>> gnssExactSyncPtr_;  // ROS Exact Sync Policy Message Filter
  // TF
  tf::TransformListener listener_;

  // Publishers
  // Odometry
  ros::Publisher pubEstOdomImu_;
  ros::Publisher pubEstMapImu_;
  // Path
  ros::Publisher pubEstOdomImuPath_;
  ros::Publisher pubEstMapImuPath_;
  ros::Publisher pubMeasMapGnssLPath_;
  ros::Publisher pubMeasMapGnssRPath_;
  ros::Publisher pubMeasMapLidarPath_;
  // TF
  tf::TransformBroadcaster tfBroadcaster_;

  // Messages
  // Odometry
  nav_msgs::OdometryPtr odomImuMsgPtr_;
  nav_msgs::OdometryPtr mapImuMsgPtr_;
  // Path
  nav_msgs::PathPtr estOdomImuPathPtr_;
  nav_msgs::PathPtr estMapImuPathPtr_;
  nav_msgs::PathPtr measMapLeftGnssPathPtr_;
  nav_msgs::PathPtr measMapRightGnssPathPtr_;
  nav_msgs::PathPtr measMapLidarPathPtr_;
};
}  // namespace excavator_se
#endif  // end M545ESTIMATORGRAPH_H
