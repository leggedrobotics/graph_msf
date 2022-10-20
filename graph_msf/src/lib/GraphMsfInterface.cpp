/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Package
#include "graph_msf/GraphMsfInterface.h"
#include "graph_msf/GraphMsf.h"

namespace graph_msf {

// Public -------------------------------------------------------------------------------------
GraphMsfInterface::GraphMsfInterface() {
  std::cout << YELLOW_START << "GMsf-Interface" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

// Protected ------------------------------------------------------------------------------------
bool GraphMsfInterface::setup_() {
  std::cout << YELLOW_START << "GMsf-Interface" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  if (!graphConfigPtr_ || !staticTransformsPtr_) {
    std::runtime_error("GMsf-Interface::setup_(): graphConfigPtr_ or staticTransformsPtr_ is not set.");
  }

  graphMsfPtr_ = std::make_shared<GraphMsf>();
  graphMsfPtr_->setup(graphConfigPtr_, staticTransformsPtr_);

  std::cout << YELLOW_START << "GMsf-Interface" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool GraphMsfInterface::initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                                            const std::string& frame2) {
  return graphMsfPtr_->initYawAndPosition(yaw_W_frame1, frame1, W_t_W_frame2, frame2);
}

bool GraphMsfInterface::initYawAndPosition_(const Eigen::Matrix4d& T_W_frame, const std::string& frameName) {
  return graphMsfPtr_->initYawAndPosition(T_W_frame, frameName);
}

bool GraphMsfInterface::areYawAndPositionInited_() {
  return graphMsfPtr_->yawAndPositionInited();
}

void GraphMsfInterface::activateFallbackGraph() {
  graphMsfPtr_->activateFallbackGraph();
}

void GraphMsfInterface::addImuMeasurementAndPublishState_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                                          const double imuTimeK) {
  static int imuCabinCallbackCounter__ = -1;

  ++imuCabinCallbackCounter__;

  std::shared_ptr<InterfacePrediction> predictionPtr;

  bool success = graphMsfPtr_->addImuMeasurement(linearAcc, angularVel, imuTimeK, predictionPtr);

  if (success) {
    publishStateAndMeasureTime_(imuTimeK, predictionPtr->T_W_O, predictionPtr->T_O_Ik, predictionPtr->I_v_W_I, predictionPtr->I_w_W_I);
  }
}

void GraphMsfInterface::addOdometryMeasurement_(const BinaryMeasurement6D& delta) {
  graphMsfPtr_->addOdometryMeasurement(delta);
}

void GraphMsfInterface::addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary) {
  graphMsfPtr_->addUnaryPoseMeasurement(unary);
}

void GraphMsfInterface::addDualOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                                    const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  graphMsfPtr_->addDualOdometryMeasurement(odometryKm1, odometryK, poseBetweenNoise);
}

void GraphMsfInterface::addDualGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                                        const Eigen::Vector3d& estCovarianceXYZ) {
  graphMsfPtr_->addDualGnssPositionMeasurement(W_t_W_frame, lastPosition, estCovarianceXYZ);
}

void GraphMsfInterface::addGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame) {
  graphMsfPtr_->addGnssPositionMeasurement(W_t_W_frame);
}

void GraphMsfInterface::addGnssHeadingMeasurement_(const UnaryMeasurement1D& yaw_W_frame) {
  graphMsfPtr_->addGnssHeadingMeasurement(yaw_W_frame);
}

void GraphMsfInterface::publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                                    const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) {
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  startLoopTime = std::chrono::high_resolution_clock::now();
  publishState_(imuTimeK, T_W_O, T_O_Ik, I_v_W_I, I_w_W_I);
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count() > (1e6 / graphConfigPtr_->imuRate / 2.0)) {
    std::cout << YELLOW_START << "GMsf-Interface" << RED_START << " Publishing state took "
              << std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count()
              << " microseconds, which is slower than double IMU-rate (" << (1e6 / graphConfigPtr_->imuRate / 2.0) << " microseconds)."
              << COLOR_END << std::endl;
  }
}

bool GraphMsfInterface::isInNormalOperation() const {
  return graphMsfPtr_->getNormalOperationFlag();
}

}  // namespace graph_msf
