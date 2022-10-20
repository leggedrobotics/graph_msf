/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

// ROS
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "urdf/model.h"

// Workspace
#include "graph_msf/StaticTransforms.h"
#include "graph_msf_ros/extrinsics/ElementToRoot.h"

namespace graph_msf {

class StaticTransformsUrdf : public StaticTransforms {
 public:
  StaticTransformsUrdf(ros::NodeHandle& privateNode);
  void setup();

 protected:
  // Names
  /// Description
  std::string urdfDescriptionName_;

  // Robot Models
  urdf::Model urdfModel_;
  std::unique_ptr<urdf::Model> model_;
  KDL::Tree tree_;

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, ElementToRoot> segments_;

  // Methods
  void getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName = "");
  tf::Transform kdlToTransform(const KDL::Frame& k);

 private:
  virtual void findTransformations() = 0;
  ros::NodeHandle privateNode_;
};
}  // namespace graph_msf
#endif  // end StaticTransformsUrdf_H
