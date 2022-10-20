/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <iostream>
// Package
#include "graph_msf/gnss/GnssHandler.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
GnssHandler::GnssHandler() {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Created Gnss Handler instance." << COLOR_END << std::endl;
}

void GnssHandler::initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates) {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initializing the handler." << COLOR_END << std::endl;

  // Initialize Gnss converter
  if (usingGnssReferenceFlag) {
    gnssSensor_.setReference(gnssReferenceLatitude_, gnssReferenceLongitude_, gnssReferenceAltitude_, gnssReferenceHeading_);
  } else {
    gnssSensor_.setReference(accumulatedLeftCoordinates(0), accumulatedLeftCoordinates(1), accumulatedLeftCoordinates(2), 0.0);
  }

  // Get Positions
  Eigen::Vector3d leftPosition, rightPosition;
  convertNavSatToPositions(accumulatedLeftCoordinates, accumulatedRightCoordinates, leftPosition, rightPosition);

  // Get heading (assuming that connection between antennas is perpendicular to heading)
  Eigen::Vector3d W_t_heading = computeHeading_(leftPosition, rightPosition);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Heading read from the Gnss is the following: " << W_t_heading << std::endl;

  // Get initial global yaw
  globalAttitudeYaw_ = computeYawFromHeadingVector_(W_t_heading);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initial global yaw is: " << 180 / M_PI * globalAttitudeYaw_ << std::endl;

  // Initial Gnss position
  W_t_W_GnssL0_ = leftPosition;
}

void GnssHandler::convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                           Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gpsToCartesian(leftGnssCoordinate(0), leftGnssCoordinate(1), leftGnssCoordinate(2));

  /// Right
  rightPosition = gnssSensor_.gpsToCartesian(rightGnssCoordinate(0), rightGnssCoordinate(1), rightGnssCoordinate(2));
}

// Heading is defined as the orthogonal vector pointing from gnssPos2 to gnssPos1, projected to x,y-plane
// Hence, if left and right gnss, then gnssPos1=leftGnss, gnssPos2=rightGnss
double GnssHandler::computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2) {
  Eigen::Vector3d robotHeading = computeHeading_(gnssPos1, gnssPos2);
  return computeYawFromHeadingVector_(robotHeading);
}

// Private ------------------------------------------------------------------

// Heading is defined as the orthogonal vector pointing from gnssPos2 to gnssPos1, projected to x,y-plane
Eigen::Vector3d GnssHandler::computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2) {
  // Compute connecting unity vector
  Eigen::Vector3d W_t_GnssR_GnssL = (gnssPos1 - gnssPos2).normalized();

  // Compute forward pointing vector
  Eigen::Vector3d zUnityVector = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d W_t_heading = W_t_GnssR_GnssL.cross(zUnityVector).normalized();

  return W_t_heading;
}

double GnssHandler::computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector) {
  return atan2(headingVector(1), headingVector(0));
}

}  // namespace graph_msf
