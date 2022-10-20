/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/ImuBuffer.hpp"

// CPP
#include <iomanip>

namespace graph_msf {

// Public --------------------------------------------------------
void ImuBuffer::addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ) {
  // Check that imuBufferLength was set
  if (imuBufferLength_ < 0) {
    throw std::runtime_error("GMsfImuBuffer: imuBufferLength has to be set by the user.");
  }

  // Convert to gtsam type
  gtsam::Vector6 imuMeas;
  imuMeas << accX, accY, accZ, gyrX, gyrY, gyrZ;

  // Add to buffer
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToImuBuffer_[ts] = imuMeas;
  }
  if (ts > tLatestInBuffer_) {
    tLatestInBuffer_ = ts;
  }

  // If IMU buffer is too large, remove first element
  if (timeToImuBuffer_.size() > imuBufferLength_) {
    timeToImuBuffer_.erase(timeToImuBuffer_.begin());
  }

  if (timeToImuBuffer_.size() > imuBufferLength_) {
    std::ostringstream errorStream;
    errorStream << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " IMU Buffer has grown too large. It contains "
                << timeToImuBuffer_.size() << " measurements instead of " << imuBufferLength_ << ".";
    throw std::runtime_error(errorStream.str());
  }
}

void ImuBuffer::addToKeyBuffer(double ts, gtsam::Key key) {
  if (verboseLevel_ >= 5) {
    std::cout << YELLOW_START << "GMsf-Imu-Buffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time " << ts
              << std::endl;
  }
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToKeyBuffer_[ts] = key;
  }

  // If Key buffer is too large, remove first element
  if (timeToKeyBuffer_.size() > imuBufferLength_) {
    timeToKeyBuffer_.erase(timeToKeyBuffer_.begin());
  }

  if (timeToKeyBuffer_.size() > imuBufferLength_) {
    std::ostringstream errorStream;
    errorStream << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Key Buffer has grown too large. It contains "
                << timeToKeyBuffer_.size() << " measurements instead of " << imuBufferLength_ << ".";
    throw std::runtime_error(errorStream.str());
  }
}

void ImuBuffer::getLastTwoMeasurements(TimeToImuMap& imuMap) {
  TimeToImuMap::iterator endItr = --(timeToImuBuffer_.end());
  TimeToImuMap::iterator previousItr = --(--(timeToImuBuffer_.end()));

  // Write into IMU Map
  imuMap[previousItr->first] = previousItr->second;
  imuMap[endItr->first] = endItr->second;
}

bool ImuBuffer::getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName,
                                          const double maxSearchDeviation, const double tK) {
  std::_Rb_tree_iterator<std::pair<const double, gtsam::Key>> upperIterator;
  {
    // Read frp, IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    upperIterator = timeToKeyBuffer_.upper_bound(tK);
  }

  auto lowerIterator = upperIterator;
  --lowerIterator;

  // Keep key which is closer to tLidar
  tInGraph = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->first : upperIterator->first;
  key = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->second : upperIterator->second;
  double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation << " ms"
              << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Latest IMU timestamp: " << tLatestInBuffer_
              << ", hence absolut delay of measurement is " << 1000 * (tLatestInBuffer_ - tK) << "ms." << std::endl;
  }

  // Check for error and warn user
  if (std::abs(timeDeviation) > maxSearchDeviation) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer " << RED_START << callingName << " Time deviation at key " << key << " is "
              << 1000 * timeDeviation << " ms, being larger than admissible deviation of " << 1000 * maxSearchDeviation << " ms"
              << COLOR_END << std::endl;
    return false;
  }

  return true;
}

bool ImuBuffer::getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, TimeToImuMap::iterator& s_itr,
                                                TimeToImuMap::iterator& e_itr) {
  // Check if timestamps are in correct order
  if (ts_start >= ts_end) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " IMU Lookup Timestamps are not correct ts_start(" << std::fixed
              << ts_start << ") >= ts_end(" << ts_end << ")\n";
    return false;
  }

  // Get Iterator Belonging to ts_start
  s_itr = timeToImuBuffer_.lower_bound(ts_start);
  // Get Iterator Belonging to ts_end
  e_itr = timeToImuBuffer_.lower_bound(ts_end);

  // Check if it is first value in the buffer which means there is no value before to interpolate with
  if (s_itr == timeToImuBuffer_.begin()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Lookup requires first message of IMU buffer, cannot Interpolate back, "
                 "Lookup Start/End: "
              << std::fixed << ts_start << "/" << ts_end << ", Buffer Start/End: " << timeToImuBuffer_.begin()->first << "/"
              << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if lookup start time is ahead of buffer start time
  if (s_itr == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " IMU Lookup start time ahead latest IMU message in the buffer, lookup: " << ts_start
              << ", latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if last value is valid
  if (e_itr == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " Lookup is past IMU buffer, with lookup Start/End: " << std::fixed
              << ts_start << "/" << ts_end << " and latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    e_itr = timeToImuBuffer_.end();
    --e_itr;
  }

  // Check if two IMU messages are different
  if (s_itr == e_itr) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Not Enough IMU values between timestamps , with Start/End: " << std::fixed << ts_start << "/" << ts_end
              << ", with diff: " << ts_end - ts_start << std::endl;
    return false;
  }

  // If everything is good
  return true;
}

bool ImuBuffer::estimateAttitudeFromImu(const std::string& imuGravityDirection, gtsam::Rot3& initAttitude, double& gravityMagnitude,
                                        Eigen::Vector3d& gyrBias) {
  // Make sure that imuBuffer is long enough
  if (imuBufferLength_ < (imuRate_ * imuPoseInitWaitSecs_)) {
    throw std::runtime_error("ImuBufferLength is not large enough for initialization. Must be at least 1 second.");
  }

  // Get timestamp of first message for lookup
  if (timeToImuBuffer_.size() < (imuRate_ * imuPoseInitWaitSecs_)) {
    return false;
  } else {
    // Accumulate Acceleration part of IMU Messages
    Eigen::Vector3d initAccMean(0.0, 0.0, 0.0), initGyrMean(0.0, 0.0, 0.0);
    for (auto& itr : timeToImuBuffer_) {
      initAccMean += itr.second.head<3>();
      initGyrMean += itr.second.tail<3>();
    }

    // Average IMU measurements and set assumed gravity direction
    initAccMean /= timeToImuBuffer_.size();
    gravityMagnitude = initAccMean.norm();
    Eigen::Vector3d gUnitVec;
    if (imuGravityDirection == "up") {
      gUnitVec = Eigen::Vector3d(0.0, 0.0, 1.0);  // ROS convention
    } else if (imuGravityDirection == "down") {
      gUnitVec = Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
      throw std::runtime_error("Gravity direction must be either 'up' or 'down'.");
    }
    // Normalize gravity vectors to remove the affect of gravity magnitude from place-to-place
    initAccMean.normalize();
    initAttitude = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(initAccMean, gUnitVec));

    // Gyro
    initGyrMean /= timeToImuBuffer_.size();
    gyrBias = initGyrMean;

    // Calculate robot initial orientation using gravity vector.
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Gravity Magnitude: " << gravityMagnitude << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Mean IMU Acceleration Vector(x,y,z): " << initAccMean.transpose()
              << " - Gravity Unit Vector(x,y,z): " << gUnitVec.transpose() << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << GREEN_START
              << " Yaw/Pitch/Roll(deg): " << initAttitude.ypr().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << "  Gyro bias(x,y,z): " << initGyrMean.transpose() << std::endl;
  }
  return true;
}
}  // namespace graph_msf