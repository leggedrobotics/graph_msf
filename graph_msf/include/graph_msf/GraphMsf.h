/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_H
#define GRAPH_MSF_H

// C++
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

// Package
#include "StaticTransforms.h"
#include "graph_msf/GraphManager.hpp"
#include "graph_msf/InterfacePrediction.h"
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/geometry/math_utils.h"
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 40          // 2*singleGnssJumping = 2*20 = 40
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.1  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace graph_msf {

class GraphMsf {
 public:
  // Constructor
  GraphMsf();
  // Destructor
  ~GraphMsf(){};

  // Setup
  bool setup(std::shared_ptr<GraphConfig> graphConfigPtr, std::shared_ptr<StaticTransforms> staticTransformsPtr);

  // Required Initialization
  bool initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                          const std::string& frame2);
  bool initYawAndPosition(const Eigen::Matrix4d& T_O_frame, const std::string& frameName);
  bool initYawAndPosition(Eigen::Matrix4d T_O_I);
  bool yawAndPositionInited();

  // Graph Manipulation
  void activateFallbackGraph();

  // Adderfunctions
  /// Return
  bool addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                         std::shared_ptr<InterfacePrediction>& predictionPtr);
  /// No return
  void addOdometryMeasurement(const BinaryMeasurement6D& delta);
  void addUnaryPoseMeasurement(const UnaryMeasurement6D& unary);
  void addDualOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                  const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                      const Eigen::Vector3d& estCovarianceXYZ);
  void addGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame);
  void addGnssHeadingMeasurement(const UnaryMeasurement1D& yaw_W_frame);

  // Getters
  bool getLogPlots() { return logPlots_; }
  void getLatestOptimizedState(Eigen::Matrix4d& optState, double& time) {
    time = optTime_;
    optState = T_W_I_opt_.matrix();
  }
  bool getNormalOperationFlag() const { return normalOperationFlag_; }

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_();
  //// Initialize the graph
  void initGraph_(const double timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  gtsam::Vector3 W_t_W_Frame1_to_W_t_W_Frame2_(const gtsam::Point3& W_t_W_frame1, const std::string& frame1, const std::string& frame2,
                                               const gtsam::Rot3& R_W_frame2);
  //// Get the robot heading from the two Gnss positions
  static gtsam::Point3 getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);

  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  std::shared_ptr<GraphManager> graphMgrPtr_ = NULL;

  // Graph Config
  std::shared_ptr<GraphConfig> graphConfigPtr_ = NULL;
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = NULL;

  /// Flags
  //// Configuration
  bool usingFallbackGraphFlag_ = true;

  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool receivedOdometryFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool gnssCovarianceViolatedFlag_ = false;
  bool normalOperationFlag_ = false;

  /// Times
  double imuTimeKm1_;
  double imuTimeOffset_ = 0.0;

  /// Transformations with timestamps
  /// Pose
  gtsam::Pose3 T_W_Ik_;
  gtsam::Pose3 T_W_O_;
  /// Velocity
  gtsam::Vector3 I_v_W_I_;
  gtsam::Vector3 I_w_W_I_;
  /// Timestamp
  double imuTimeK_;
  /// Other
  gtsam::Pose3 T_W_I0_;  // Initial IMU pose (in graph)
  gtsam::Pose3 T_W_I_opt_;
  double optTime_;

  /// Attitudes
  double gravityConstant_ = 9.81;  // Will be overwritten
  double yaw_W_I0_;
  gtsam::Vector3 W_t_W_I0_;
  double imuAttitudePitch_;
  double imuAttitudeRoll_;

  /// Counter
  long gnssCallbackCounter_ = 0;
  int gnssNotJumpingCounter_ = REQUIRED_GNSS_NUM_NOT_JUMPED;

  /// Logging
  bool logPlots_ = false;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_H
