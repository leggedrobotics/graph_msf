/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef IMU_MANAGER_HPP_
#define IMU_MANAGER_HPP_
#define DEFAULT_IMU_RATE 100

// C++
#include <map>

// Eigen
#include <Eigen/Dense>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// Package
#include "graph_msf/Datatypes.hpp"

namespace graph_msf {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

class ImuBuffer {
 public:
  // Constructor
  ImuBuffer() : imuRate_(DEFAULT_IMU_RATE) {
    // Reset IMU Buffer
    timeToImuBuffer_.clear();
  }

  // Destructor
  ~ImuBuffer() = default;

  // Setters
  inline void setImuRate(double d) { imuRate_ = d; }
  inline void setImuBufferLength(int i) { imuBufferLength_ = i; }
  inline void setVerboseLevel(int i) { verboseLevel_ = i; }

  // Add to buffers
  void addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ);
  void addToKeyBuffer(double ts, gtsam::Key key);

  // Getters
  inline double getImuRate() const { return imuRate_; }
  inline double getLatestTimestampInBuffer() const { return tLatestInBuffer_; }
  void getLastTwoMeasurements(TimeToImuMap& imuMap);
  bool getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName, const double maxSearchDeviation,
                                 const double tLidar);
  bool getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, TimeToImuMap::iterator& s_itr,
                                       TimeToImuMap::iterator& e_itr);

  // Public member functions
  /// Determine initial IMU pose w.r.t to gravity vector pointing up
  bool estimateAttitudeFromImu(const std::string& imuGravityDirection, gtsam::Rot3& init_attitude, double& gravity_magnitude,
                               Eigen::Vector3d& gyrBias);

 private:
  // Member variables
  TimeToImuMap timeToImuBuffer_;  // IMU buffer
  TimeToKeyMap timeToKeyBuffer_;
  double imuRate_ = -1;  // Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  int imuBufferLength_ = -1;
  const double imuPoseInitWaitSecs_ = 1.0;  // Multiplied with _imuRate
  int verboseLevel_ = 0;
  double tLatestInBuffer_ = 0.0;
  std::mutex writeInBufferMutex_;
};

}  // namespace graph_msf

#endif  // IMU_MANAGER_HPP_