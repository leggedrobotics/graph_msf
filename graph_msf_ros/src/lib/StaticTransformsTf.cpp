/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

// ROS
#include <ros/ros.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

StaticTransformsTf::StaticTransformsTf(ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

}  // namespace graph_msf
