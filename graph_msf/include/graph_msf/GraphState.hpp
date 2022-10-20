/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_STATE_HPP_
#define GRAPH_STATE_HPP_

// C++
#include <mutex>

// GTSAM
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace graph_msf {

// Class defining Robot State
class State {
 private:
  gtsam::Key key_ = 0;                    // key
  double ts_ = 0.0;                       // timestamp
  gtsam::NavState navState_;              // pose, velocity
  gtsam::imuBias::ConstantBias imuBias_;  // imu bias
  std::mutex stateMutex_;

 public:
  State(){};   // Constructor
  ~State(){};  // Destructor

  // Accessors
  const auto key() const { return key_; }
  const auto ts() const { return ts_; }
  const auto& navState() const { return navState_; }
  const auto& imuBias() const { return imuBias_; }

  // Update state Graph Key and Timestamp
  void updateKeyAndTimestamp(const gtsam::Key key, const double ts) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
  }

  // Update state Graph Key, Timestamp and NavState(Pose+Velocity)
  void updateNavState(const gtsam::Key key, const double ts, const gtsam::NavState& navState) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
  }

  // Update state Graph Key, Timestamp and IMU Bias estimate
  void updateImuBias(const gtsam::Key key, const double ts, const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    imuBias_ = imuBias;
  }

  // Update state Graph Key, Timestamp, NavState(Pose+Velocity) and IMU Bias estimate
  void updateNavStateAndBias(const gtsam::Key key, const double ts, const gtsam::NavState& navState,
                             const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
    imuBias_ = imuBias;
  }

  // Overload assignment operator
  State& operator=(State& other) {
    this->key_ = other.key_;
    this->ts_ = other.ts_;
    this->navState_ = other.navState_;
    this->imuBias_ = other.imuBias_;
    return *this;
  }
};

}  // namespace graph_msf

#endif  // GRAPH_STATE_HPP_