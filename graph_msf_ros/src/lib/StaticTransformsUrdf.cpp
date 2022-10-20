/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsUrdf.h"

namespace graph_msf {

StaticTransformsUrdf::StaticTransformsUrdf(ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "StaticTransformsUrdf" << GREEN_START << " Initializing..." << COLOR_END << std::endl;
  privateNode_ = privateNode;
}

void StaticTransformsUrdf::setup() {
  std::cout << YELLOW_START << "StaticTransformsUrdf" << GREEN_START << " Setting up for description name '" << urdfDescriptionName_ << "'."
            << COLOR_END << std::endl;

  // Check whether description contains information
  std::string urdfDescriptionContent;
  privateNode_.getParam(std::string("/") + urdfDescriptionName_, urdfDescriptionContent);
  if (urdfDescriptionContent.empty()) {
    throw std::runtime_error("StaticTransformsUrdf - Could not load description, description empty.");
  }

  // load excavator model from URDF
  double timeStep;
  urdfModel_.initParam(urdfDescriptionName_);

  // Initialize the KDL tree
  if (!kdl_parser::treeFromUrdfModel(urdfModel_, tree_)) {
    throw std::runtime_error("Failed to extract kdl tree from robot description");
  }

  KDL::SegmentMap segments_map = tree_.getSegments();
  KDL::Chain chain;

  // walk the tree and add segments to segments_
  segments_.clear();
  getRootTransformations(tree_.getRootSegment());

  // Summary
  std::cout << YELLOW_START << "StaticTransformsUrdf" << GREEN_START << " Initialized." << COLOR_END << std::endl;
}

void StaticTransformsUrdf::getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName) {
  const std::string& elementName = GetTreeElementSegment(element->second).getName();
  if (rootName == "") {
    rootName = elementName;
  }
  // Iterate through children
  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(element->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    // Go through children
    const KDL::SegmentMap::const_iterator child = children[i];
    // Get kinematic chain from current child to root
    KDL::Chain chain;
    tree_.getChain(rootName, child->second.segment.getName(), chain);
    // Compute trafo to root
    tf::Transform T_root_element = tf::Transform::getIdentity();
    for (int i = 0; i < chain.getNrOfSegments(); ++i) {
      KDL::Frame frameToRoot = chain.getSegment(i).getFrameToTip();
      double x, y, z, w;
      frameToRoot.M.GetQuaternion(x, y, z, w);
      tf::Quaternion q(x, y, z, w);
      tf::Vector3 t(frameToRoot.p[0], frameToRoot.p[1], frameToRoot.p[2]);
      T_root_element = T_root_element * tf::Transform(q, t);
    }
    // Write into buffer
    ElementToRoot elementToRoot(T_root_element, rootName, child->second.segment.getName());
    // Insert into segments
    segments_.insert(std::make_pair(child->second.segment.getName(), elementToRoot));
    // Call recursively
    getRootTransformations(child, rootName);
  }
}

tf::Transform StaticTransformsUrdf::kdlToTransform(const KDL::Frame& k) {
  tf::Transform tf_T;
  tf_T.setOrigin(tf::Vector3(k.p.x(), k.p.y(), k.p.z()));
  double qx, qy, qz, qw;
  k.M.GetQuaternion(qx, qy, qz, qw);
  tf_T.setRotation(tf::Quaternion(qx, qy, qz, qw));

  return tf_T;
}

}  // namespace graph_msf
