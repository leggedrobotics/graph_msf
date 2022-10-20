/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_dual_graph/ExcavatorEstimator.h"

namespace excavator_se {

template <typename T>
void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "ExcavatorEstimator " << COLOR_END << key << "  set to: " << value << std::endl;
}
template <>
void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "ExcavatorEstimator " << COLOR_END << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}
// Simon Kerscher: Implementation of Templating
template <typename T>
T tryGetParam(const std::string& key, const ros::NodeHandle& privateNode) {
  T value;
  if (privateNode.getParam(key, value)) {
    printKey(key, value);
    return value;
  } else {
    throw std::runtime_error("ExcavatorEstimator - " + key + " not specified.");
  }
}

void ExcavatorEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  if (!graphConfigPtr_) {
    throw std::runtime_error("ExcavatorEstimator: graphConfigPtr must be initialized.");
  }

  // Configuration
  /// Using Gnss
  graphConfigPtr_->usingGnssFlag = tryGetParam<bool>("launch/usingGnss", privateNode);

  // Set frames
  /// Map
  std::string frame = tryGetParam<std::string>("extrinsics/mapFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setMapFrame(frame);
  /// Odom
  frame = tryGetParam<std::string>("extrinsics/odomFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setOdomFrame(frame);
  /// base_link
  frame = tryGetParam<std::string>("extrinsics/baseLinkFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setBaseLinkFrame(frame);
  /// IMU
  //// Cabin IMU
  frame = tryGetParam<std::string>("extrinsics/imuFrame", privateNode);
  staticTransformsPtr_->setImuFrame(frame);
  /// LiDAR frame
  frame = tryGetParam<std::string>("extrinsics/lidarFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLidarFrame(frame);
  /// Cabin frame
  frame = tryGetParam<std::string>("extrinsics/cabinFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setCabinFrame(frame);
  /// Left Gnss frame
  frame = tryGetParam<std::string>("extrinsics/gnssFrame1", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLeftGnssFrame(frame);
  /// Right Gnss frame
  frame = tryGetParam<std::string>("extrinsics/gnssFrame2", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setRightGnssFrame(frame);

  // IMU gravity definition
  graphConfigPtr_->imuGravityDirection = tryGetParam<std::string>("launch/imuGravityDirection", privateNode);

  // Factor Graph Parameters
  graphConfigPtr_->imuRate = tryGetParam<double>("sensor_params/imuRate", privateNode);
  graphConfigPtr_->maxSearchDeviation = 1.0 / graphConfigPtr_->imuRate;
  graphConfigPtr_->imuBufferLength = tryGetParam<double>("sensor_params/imuBufferLength", privateNode);
  lidarRate_ = tryGetParam<double>("sensor_params/lidarOdometryRate", privateNode);
  gnssLeftRate_ = tryGetParam<double>("sensor_params/gnssRate", privateNode);
  gnssRightRate_ = tryGetParam<double>("sensor_params/gnssRate", privateNode);
  graphConfigPtr_->imuTimeOffset = tryGetParam<double>("sensor_params/imuTimeOffset", privateNode);
  graphConfigPtr_->useIsam = tryGetParam<bool>("graph_params/useIsam", privateNode);
  graphConfigPtr_->smootherLag = tryGetParam<double>("graph_params/smootherLag", privateNode);

  // TODO --> replace
  graphConfigPtr_->additionalIterations = tryGetParam<int>("graph_params/additionalOptimizationIterations", privateNode);
  graphConfigPtr_->findUnusedFactorSlots = tryGetParam<bool>("graph_params/findUnusedFactorSlots", privateNode);
  graphConfigPtr_->enableDetailedResults = tryGetParam<bool>("graph_params/enableDetailedResults", privateNode);
  graphConfigPtr_->usingFallbackGraphFlag = tryGetParam<bool>("graph_params/usingFallbackGraph", privateNode);

  // Outlier Parameters
  graphConfigPtr_->gnssOutlierThresold = tryGetParam<double>("outlier_params/gnssOutlierThreshold", privateNode);

  // Noise Parameters
  /// Accelerometer

  graphConfigPtr_->accNoiseDensity = tryGetParam<double>("noise_params/accNoiseDensity", privateNode);
  graphConfigPtr_->accBiasRandomWalk = tryGetParam<double>("noise_params/accBiasRandomWalk", privateNode);
  {
    const double accBiasPrior = tryGetParam<double>("noise_params/accBiasPrior", privateNode);
    graphConfigPtr_->accBiasPrior = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);
  }
  /// Gyro
  graphConfigPtr_->gyroNoiseDensity = tryGetParam<double>("noise_params/gyrNoiseDensity", privateNode);
  graphConfigPtr_->gyroBiasRandomWalk = tryGetParam<double>("noise_params/gyrBiasRandomWalk", privateNode);
  {
    const double gyroBiasPrior = tryGetParam<double>("noise_params/gyrBiasPrior", privateNode);
    graphConfigPtr_->gyroBiasPrior = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);
  }
  /// Preintegration
  graphConfigPtr_->integrationNoiseDensity = tryGetParam<double>("noise_params/integrationNoiseDensity", privateNode);
  graphConfigPtr_->biasAccOmegaPreint = tryGetParam<double>("noise_params/biasAccOmegaPreInt", privateNode);
  /// LiDAR Odometry
  {
    const auto poseBetweenNoise = tryGetParam<std::vector<double>>("noise_params/poseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
    poseBetweenNoise_ << poseBetweenNoise[0], poseBetweenNoise[1], poseBetweenNoise[2], poseBetweenNoise[3], poseBetweenNoise[4],
        poseBetweenNoise[5];
  }
  {
    const auto poseUnaryNoise = tryGetParam<std::vector<double>>("noise_params/poseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
    poseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  }
  /// Gnss
  gnssPositionUnaryNoise_ = tryGetParam<double>("noise_params/gnssPositionUnaryNoise", privateNode);
  gnssHeadingUnaryNoise_ = tryGetParam<double>("noise_params/gnssHeadingUnaryNoise", privateNode);

  // Relinearization
  graphConfigPtr_->positionReLinTh = tryGetParam<double>("relinearization_params/positionReLinTh", privateNode);
  graphConfigPtr_->rotationReLinTh = tryGetParam<double>("relinearization_params/rotationReLinTh", privateNode);
  graphConfigPtr_->velocityReLinTh = tryGetParam<double>("relinearization_params/velocityReLinTh", privateNode);
  graphConfigPtr_->accBiasReLinTh = tryGetParam<double>("relinearization_params/accBiasReLinTh", privateNode);
  graphConfigPtr_->gyroBiasReLinTh = tryGetParam<double>("relinearization_params/gyrBiasReLinTh", privateNode);
  graphConfigPtr_->relinearizeSkip = tryGetParam<int>("relinearization_params/relinearizeSkip", privateNode);
  graphConfigPtr_->enableRelinearization = tryGetParam<bool>("relinearization_params/enableRelinearization", privateNode);
  graphConfigPtr_->evaluateNonlinearError = tryGetParam<bool>("relinearization_params/evaluateNonlinearError", privateNode);
  graphConfigPtr_->cacheLinearizedFactors = tryGetParam<bool>("relinearization_params/cacheLinearizedFactors", privateNode);
  graphConfigPtr_->enablePartialRelinearizationCheck =
      tryGetParam<bool>("relinearization_params/enablePartialRelinearizationCheck", privateNode);

  // Common Parameters
  verboseLevel_ = tryGetParam<int>("common_params/verbosity", privateNode);
  graphConfigPtr_->verboseLevel = verboseLevel_;

  if (graphConfigPtr_->usingGnssFlag) {
    // Gnss parameters
    gnssHandlerPtr_->usingGnssReferenceFlag = tryGetParam<bool>("gnss/useGnssReference", privateNode);
    {
      const double referenceLatitude = tryGetParam<double>("gnss/referenceLatitude", privateNode);
      gnssHandlerPtr_->setGnssReferenceLatitude(referenceLatitude);
    }
    {
      const double referenceLongitude = tryGetParam<double>("gnss/referenceLongitude", privateNode);
      gnssHandlerPtr_->setGnssReferenceLongitude(referenceLongitude);
    }
    {
      const double referenceAltitude = tryGetParam<double>("gnss/referenceAltitude", privateNode);
      gnssHandlerPtr_->setGnssReferenceAltitude(referenceAltitude);
    }
    {
      const double referenceHeading = tryGetParam<double>("gnss/referenceHeading", privateNode);
      gnssHandlerPtr_->setGnssReferenceHeading(referenceHeading);
    }
  }
}

}  // namespace excavator_se
