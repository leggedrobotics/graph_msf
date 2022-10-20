/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_dual_graph/ExcavatorStaticTransforms.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace excavator_se {

ExcavatorStaticTransforms::ExcavatorStaticTransforms(std::shared_ptr<ros::NodeHandle> privateNodePtr)
    : graph_msf::StaticTransformsTf(*privateNodePtr) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void ExcavatorStaticTransforms::findTransformations() {
  // Print to console --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree.";
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " " << lidarFrame_ << ", " << leftGnssFrame_ << ", " << rightGnssFrame_
            << ", " << cabinFrame_ << ", " << imuFrame_ << ", " << baseLinkFrame_ << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Cabin Link ---
  listener_.waitForTransform(imuFrame_, cabinFrame_, ros::Time(0), ros::Duration(100.0));
  listener_.lookupTransform(imuFrame_, cabinFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, cabinFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Cabin: " << rv_T_frame1_frame2(imuFrame_, cabinFrame_).block<3, 1>(0, 3) << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(cabinFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, cabinFrame_).inverse();

  // Imu to Base Link ---
  listener_.waitForTransform(imuFrame_, baseLinkFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, baseLinkFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Base: " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).block<3, 1>(0, 3) << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Imu to LiDAR Link ---
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(imuFrame_, lidarFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, lidarFrame_, ros::Time(0), transform);
  // I_Lidar
  graph_msf::tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, lidarFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Lidar: " << rv_T_frame1_frame2(imuFrame_, lidarFrame_).block<3, 1>(0, 3) << std::endl;
  // Lidar_I
  lv_T_frame1_frame2(lidarFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarFrame_).inverse();

  // Imu to GNSS Left Link ---
  listener_.waitForTransform(imuFrame_, leftGnssFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, leftGnssFrame_, ros::Time(0), transform);
  // I_GnssL
  graph_msf::tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, leftGnssFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_GnssL: " << rv_T_frame1_frame2(imuFrame_, leftGnssFrame_).block<3, 1>(0, 3) << std::endl;
  // GnssL_I
  lv_T_frame1_frame2(leftGnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, leftGnssFrame_).inverse();

  // Imu to GNSS Right Link ---
  listener_.waitForTransform(imuFrame_, rightGnssFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, rightGnssFrame_, ros::Time(0), transform);
  // I_GnssR
  graph_msf::tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, rightGnssFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_GnssR: " << rv_T_frame1_frame2(imuFrame_, rightGnssFrame_).block<3, 1>(0, 3) << std::endl;
  // GnssL_I
  lv_T_frame1_frame2(rightGnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, rightGnssFrame_).inverse();

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace excavator_se
