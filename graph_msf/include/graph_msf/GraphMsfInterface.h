/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF__INTERFACE_H
#define GRAPH_MSF__INTERFACE_H

#include <Eigen/Eigen>
#include <thread>

// Workspace
#include "StaticTransforms.h"
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"

namespace graph_msf {

class GraphMsf;

class GraphMsfInterface {
 public:
  GraphMsfInterface();

 protected:
  // Setup
  bool setup_();

  // Required for initialization
  bool initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                           const std::string& frame2);
  bool initYawAndPosition_(const Eigen::Matrix4d& T_W_frame, const std::string& frameName);
  bool areYawAndPositionInited_();

  // Graph Maniupulation

  void activateFallbackGraph();

  // Write measurements
  void addImuMeasurementAndPublishState_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK);
  void addOdometryMeasurement_(const BinaryMeasurement6D& delta);
  void addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary);
  void addDualOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                   const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addDualGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                       const Eigen::Vector3d& estCovarianceXYZ);
  void addGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame);
  void addGnssHeadingMeasurement_(const UnaryMeasurement1D& yaw_W_frame);

  // Publish
  virtual void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                             const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) = 0;

  // Getters
  bool isInNormalOperation() const;

  // Member Variables
  /// GraphMsf
  std::shared_ptr<GraphMsf> graphMsfPtr_ = NULL;
  /// Graph Configuration
  std::shared_ptr<GraphConfig> graphConfigPtr_ = NULL;
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = NULL;

  /// Verbosity
  int verboseLevel_ = 0;

 private:
  // Threads
  std::thread publishStateThread_;

  // Functions
  void publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                   const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I);
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_INTERFACE_H
