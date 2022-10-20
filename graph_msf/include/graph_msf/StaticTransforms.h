/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef STATIC_TRANSFORMS_H
#define STATIC_TRANSFORMS_H

#include <Eigen/Eigen>
#include <iostream>

namespace graph_msf {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

class StaticTransforms {
 public:
  // Constructor
  StaticTransforms() { std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl; }

  // Setters
  Eigen::Matrix4d& lv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    //    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    //    if (keyIterator == T_frame1_frame2_map_.end()) {
    //      std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No transform found for " << frame1 << " and " << frame2
    //                << std::endl;
    //      throw std::runtime_error("No transform found for " + frame1 + " and " + frame2);
    //    }
    return T_frame1_frame2_map_[framePair];
  }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }

  // Getters
  const Eigen::Matrix4d& rv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    if (keyIterator == T_frame1_frame2_map_.end()) {
      std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No transform found for " << frame1 << " and " << frame2
                << std::endl;
      throw std::runtime_error("No transform found for " + frame1 + " and " + frame2);
    }
    return keyIterator->second;
  }

  const std::string& getImuFrame() { return imuFrame_; }

  // Functionality
  virtual void findTransformations() = 0;

 protected:
  // General container class
  std::map<std::pair<std::string, std::string>, Eigen::Matrix4d> T_frame1_frame2_map_;

  // Required frames
  std::string imuFrame_;  // If used --> copied to imuCabinFrame_
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
