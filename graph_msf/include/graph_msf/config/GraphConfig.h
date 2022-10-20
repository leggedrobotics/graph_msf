#ifndef GRAPHCONFIG_H
#define GRAPHCONFIG_H

#include <Eigen/Eigen>

namespace graph_msf {

struct GraphConfig {
  GraphConfig() {}

  // Strings
  std::string imuGravityDirection = "up";

  // Booleans
  /// ISAM
  bool useIsam = true;
  bool findUnusedFactorSlots = false;
  bool enableDetailedResults = false;
  bool enableRelinearization = true;
  bool evaluateNonlinearError = true;
  bool cacheLinearizedFactors = true;
  bool enablePartialRelinearizationCheck = true;
  /// Flags
  bool usingFallbackGraphFlag = true;
  bool usingGnssFlag = true;
  bool usingLioFlag = true;

  // Integers
  int verboseLevel = 0;
  int imuBufferLength = 200;
  int additionalIterations = 0;
  int relinearizeSkip = 0;

  // Doubles
  double imuRate = 100;
  double maxSearchDeviation = 0.01;
  double imuTimeOffset = 0.0;
  double smootherLag = 6.0;
  double zeroMotionTh = 0.0;
  double gnssOutlierThresold = 0.3;
  /// Noise Params
  double accNoiseDensity = 1e-8;
  double accBiasRandomWalk = 1e-8;
  double gyroNoiseDensity = 1e-8;
  double gyroBiasRandomWalk = 1e-8;
  double integrationNoiseDensity = 1.0;
  double biasAccOmegaPreint = 1.0;
  /// Optimization
  double positionReLinTh = 1e-3;
  double rotationReLinTh = 1e-3;
  double velocityReLinTh = 1e-3;
  double accBiasReLinTh = 1e-3;
  double gyroBiasReLinTh = 1e-3;

  // Eigen
  Eigen::Vector3d accBiasPrior = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d gyroBiasPrior = Eigen::Vector3d(0.0, 0.0, 0.0);
};

}  // namespace graph_msf

#endif  // GRAPHCONFIG_H
