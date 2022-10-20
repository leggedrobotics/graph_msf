/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/GraphMsf.h"

namespace graph_msf {

// Public -----------------------------------------------------------
/// Constructor -----------
GraphMsf::GraphMsf() {
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

/// Setup ------------
bool GraphMsf::setup(std::shared_ptr<GraphConfig> graphConfigPtr, std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Graph Config
  graphConfigPtr_ = graphConfigPtr;
  staticTransformsPtr_ = staticTransformsPtr;

  graphMgrPtr_ = std::make_shared<GraphManager>(graphConfigPtr_);

  // Configs
  graphMgrPtr_->getIsamParamsReference().findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlots;
  graphMgrPtr_->getIsamParamsReference().setEnableDetailedResults(graphConfigPtr_->enableDetailedResults);
  graphMgrPtr_->getIsamParamsReference().setRelinearizeSkip(graphConfigPtr_->relinearizeSkip);
  graphMgrPtr_->getIsamParamsReference().setEnableRelinearization(graphConfigPtr_->enableRelinearization);
  graphMgrPtr_->getIsamParamsReference().setEvaluateNonlinearError(graphConfigPtr_->evaluateNonlinearError);
  graphMgrPtr_->getIsamParamsReference().setCacheLinearizedFactors(graphConfigPtr_->cacheLinearizedFactors);
  graphMgrPtr_->getIsamParamsReference().setEnablePartialRelinearizationCheck(graphConfigPtr_->enablePartialRelinearizationCheck);

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&GraphMsf::optimizeGraph_, this);
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool GraphMsf::yawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

void GraphMsf::activateFallbackGraph() {
  if (graphConfigPtr_->usingFallbackGraphFlag) {
    graphMgrPtr_->activateFallbackGraph();
  } else {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Not activating fallback graph, disabled in config." << COLOR_END << std::endl;
  }
}

bool GraphMsf::initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                                  const std::string& frame2) {
  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_W_frame1);
  gtsam::Rot3 yawR_W_I0;
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Setting yaw in " << frame1 << " frame." << COLOR_END << std::endl;
  yawR_W_I0 =
      yawR_W_frame1 * gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(frame1, staticTransformsPtr_->getImuFrame())).rotation();

  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (not alignedImuFlag_) {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  } else if (not yawAndPositionInited()) {
    yaw_W_I0_ = yawR_W_I0.yaw();
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Initial global yaw of imu in world has been set to (deg) "
              << 180.0 * yaw_W_I0_ / M_PI << "." << COLOR_END << std::endl;

    gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
    // Set Member Variables
    W_t_W_I0_ = W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);

    foundInitialYawAndPositionFlag_ = true;
    return true;
  } else {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END << std::endl;
    return false;
  }
}

bool GraphMsf::initYawAndPosition(const Eigen::Matrix4d& T_O_frame, const std::string& frameName) {
  gtsam::Pose3 T_O_frame_gtsam(T_O_frame);
  return initYawAndPosition(T_O_frame_gtsam.rotation().yaw(), frameName, T_O_frame.block<3, 1>(0, 3), frameName);
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
bool GraphMsf::addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                 std::shared_ptr<InterfacePrediction>& predictionPtr) {
  // Static Members
  static gtsam::Pose3 T_W_I_km1__;
  static int imuCabinCallbackCounter__ = -1;

  // Increase counter
  ++imuCabinCallbackCounter__;

  double imuTimeKm1 = imuTimeK_;
  imuTimeK_ = imuTimeK;

  // Filter out imu messages with same time stamp
  if (std::abs(imuTimeK_ - imuTimeKm1) < 1e-8) {
    std::cout << YELLOW_START << " GMsf" << RED_START << " Imu time " << std::setprecision(14) << imuTimeK << " was repeated." << COLOR_END
              << std::endl;
    return false;
  }

  // Add measurement to buffer
  graphMgrPtr_->addToIMUBuffer(imuTimeK, linearAcc, angularVel);

  // Variable of odometry
  gtsam::Pose3 T_O_Ik;
  gtsam::NavState T_W_Ik_nav;

  // Loop variables
  bool relocalizationFlag = false;  // Edited by graphMgrPtr_->addImuFactorAndGetState

  // If IMU not yet gravity aligned
  if (!alignedImuFlag_) {  // Not yet gravity aligned
    // Try to align
    if (!alignImu_()) {
      // Print only once per second
      if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
        std::cout << YELLOW_START << "GMsf" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..." << std::endl;
      }
      return false;
    } else {
      gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(0.0, imuAttitudePitch_, imuAttitudeRoll_);
      T_W_Ik_ = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0));
      T_W_O_ = gtsam::Pose3();
      I_v_W_I_ = gtsam::Vector3(0.0, 0.0, 0.0);
      I_w_W_I_ = gtsam::Vector3(0.0, 0.0, 0.0);
      alignedImuFlag_ = true;

      return false;
    }
  } else if (!yawAndPositionInited()) {  // Gravity Aligned but not yaw-aligned
    if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << YELLOW_START << "GMsf" << COLOR_END << " IMU callback waiting for initialization of global yaw and initial position."
                << std::endl;
    }
    T_W_Ik_nav = gtsam::NavState(T_W_Ik_, I_v_W_I_);
  } else if (!initedGraphFlag_) {  // Initialization
    initGraph_(imuTimeK);
    //    if (!graphConfigPtr_->usingGnssFlag) {
    //      // TODO
    //      //activateFallbackGraph();
    //    }
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    T_W_Ik_nav = gtsam::NavState(T_W_Ik_, I_v_W_I_);
    relocalizationFlag = true;
  } else {  // Normal operation
    normalOperationFlag_ = true;
    // Add IMU factor and get propagated state
    T_W_Ik_nav = graphMgrPtr_->addImuFactorAndGetState(imuTimeK, linearAcc, angularVel, relocalizationFlag);
  }

  // Assign poses and velocities ---------------------------------------------------
  T_W_Ik_ = T_W_Ik_nav.pose();
  I_v_W_I_ = T_W_Ik_nav.bodyVelocity();
  I_w_W_I_ = graphMgrPtr_->getIMUBias().correctGyroscope(angularVel);
  imuAttitudeRoll_ = T_W_Ik_.rotation().roll();
  imuAttitudePitch_ = T_W_Ik_.rotation().pitch();

  // If relocalization happens --> write to map->odom ------------------------------------
  if (relocalizationFlag) {
    // Print
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Relocalization is needed. Publishing to map->odom." << COLOR_END << std::endl;
    // For this computation step assume T_O_Ik ~ T_O_Ikm1
    gtsam::Pose3 T_I_O_km1 = T_W_I_km1__.inverse() * T_W_O_;
    T_W_O_ = T_W_Ik_ * T_I_O_km1;
  }

  // Convert to odom frame ----------------------------------------------------------------
  T_O_Ik = T_W_O_.inverse() * T_W_Ik_;

  // Estimate state
  // if (receivedOdometryFlag_) {}

  // Publish
  predictionPtr =
      std::make_shared<InterfacePrediction>(T_W_O_.matrix(), T_O_Ik.matrix(), Eigen::Vector3d(I_v_W_I_), Eigen::Vector3d(I_w_W_I_));

  // Write for next iteration
  T_W_I_km1__ = T_W_Ik_;
  return true;
}

void GraphMsf::addOdometryMeasurement(const BinaryMeasurement6D& delta) {}  // TODO

void GraphMsf::addUnaryPoseMeasurement(const UnaryMeasurement6D& unary) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  gtsam::Pose3 T_W_frame(unary.measurementPose());
  gtsam::Pose3 T_W_I =
      T_W_frame * gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(unary.frameName(), staticTransformsPtr_->getImuFrame()));

  if (initedGraphFlag_) {
    graphMgrPtr_->addPoseUnaryFactorToGlobalGraph(unary.timeK(), unary.measurementRate(), unary.measurementNoise(), T_W_I);
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(unary.timeK(), unary.measurementRate(), unary.measurementNoise(), T_W_I);

    // Optimize
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

void GraphMsf::addDualOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                          const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static Eigen::Matrix4d T_Wl_Lj__;
  static gtsam::Pose3 T_W_Ij_Graph__;
  static gtsam::Key lastDeltaMeasurementKey__;

  // Create Pseudo Unary Factor
  if (graphMgrPtr_->globalGraphActiveFlag()) {
    /// Reset LiDAR Unary factor intiialization
    lidarUnaryFactorInitialized__ = false;
    // Write current odometry pose to latest delta pose
    T_Wl_Lj__ = odometryK.measurementPose();
  }  //
  else if (graphMgrPtr_->fallbackGraphActiveFlag()) {
    if (!lidarUnaryFactorInitialized__) {
      // Calculate state still from globalGraph
      T_W_Ij_Graph__ = graphMgrPtr_->calculateActiveStateAtKey(lastDeltaMeasurementKey__).pose();
      std::cout << YELLOW_START << "GMsf" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
      lidarUnaryFactorInitialized__ = true;
    }
    /// Delta pose
    gtsam::Pose3 T_Ij_Ik(staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), odometryKm1.frameName()) *
                         T_Wl_Lj__.inverse() * odometryK.measurementPose() *
                         staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame()));
    gtsam::Pose3 pseudo_T_W_Ik = T_W_Ij_Graph__ * T_Ij_Ik;
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(odometryK.timeK(), odometryK.measurementRate(), odometryK.measurementNoise(),
                                                    pseudo_T_W_Ik);
  }
  /// Delta pose
  Eigen::Matrix4d T_Lkm1_Lk = odometryKm1.measurementPose().inverse() * odometryK.measurementPose();
  Eigen::Matrix4d T_Ikm1_Ik = staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), odometryKm1.frameName()) *
                              T_Lkm1_Lk *
                              staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame());
  lastDeltaMeasurementKey__ = graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(
      odometryKm1.timeK(), odometryK.timeK(), odometryK.measurementRate(), poseBetweenNoise, gtsam::Pose3(T_Ikm1_Ik));

  // Optimize
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }

  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

void GraphMsf::addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& W_t_W_frame_km1,
                                              const Eigen::Vector3d& estCovarianceXYZ) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Read covariance
  bool gnssCovarianceViolatedFlag = estCovarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    estCovarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    estCovarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
  if (gnssCovarianceViolatedFlag && !gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss measurments now ABSENT due to too big covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = true;
  } else if (!gnssCovarianceViolatedFlag && gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss returned. Low covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = false;
  }

  // Gnss jumping?
  double jumpingDistance = (W_t_W_frame_km1 - W_t_W_frame.measurementVector()).norm();
  if (jumpingDistance < graphConfigPtr_->gnssOutlierThresold) {  // gnssOutlierThreshold_) {
    ++gnssNotJumpingCounter_;
    if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss was not jumping recently. Jumping counter valid again." << COLOR_END
                << std::endl;
    }
  } else {
    if (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss was jumping: Distance is " << jumpingDistance << "m, larger than allowed "
                << 1.0  // gnssOutlierThreshold_
                << "m.  Reset outlier counter." << COLOR_END << std::endl;
    }
    gnssNotJumpingCounter_ = 0;
  }

  // Case: Gnss is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor
    addGnssPositionMeasurement(W_t_W_frame);
    graphMgrPtr_->activateGlobalGraph();
  }
  // Case: Gnss is bad --> Do not write to graph, set flags for odometry unary factor to true
  else if (usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }
}

void GraphMsf::addGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Add unary factor
  gtsam::Point3 W_t_W_I = W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame.measurementVector(), W_t_W_frame.frameName(),
                                                        staticTransformsPtr_->getImuFrame(), T_W_Ik_.rotation());
  if (graphMgrPtr_->getStateKey() == 0) {
    return;
  }
  graphMgrPtr_->addGnssPositionUnaryFactor(W_t_W_frame.timeK(), W_t_W_frame.measurementRate(), W_t_W_frame.measurementNoise(), W_t_W_I);
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

void GraphMsf::addGnssHeadingMeasurement(const UnaryMeasurement1D& yaw_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(yaw_W_frame.measurementValue());
  gtsam::Rot3 yawR_W_I0 =
      yawR_W_frame *
      gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(yaw_W_frame.frameName(), staticTransformsPtr_->getImuFrame())).rotation();

  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    graphMgrPtr_->addGnssHeadingUnaryFactor(yaw_W_frame.timeK(), yaw_W_frame.measurementRate(), yaw_W_frame.measurementNoise(),
                                            yawR_W_I0.yaw());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

/// Worker Functions -----------------------
bool GraphMsf::alignImu_() {
  gtsam::Rot3 imuAttitude;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  if (graphMgrPtr_->estimateAttitudeFromImu(graphConfigPtr_->imuGravityDirection, imuAttitude, gravityConstant_,
                                            graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll_ = imuAttitude.roll();
    imuAttitudePitch_ = imuAttitude.pitch();
    std::cout << YELLOW_START << "GMsf" << COLOR_END
              << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_ << std::endl;
    return true;
  } else {
    return false;
  }
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void GraphMsf::initGraph_(const double timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Rot3 yawR_W_I0 = gtsam::Rot3::Yaw(yaw_W_I0_);
  // gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);

  std::cout << YELLOW_START << "GMsf" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0.ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(gravityConstant_, graphConfigPtr_->imuGravityDirection);
  /// Add initial IMU translation based on intial orientation
  gtsam::Pose3 T_W_I0;

  T_W_I0 = gtsam::Pose3(R_W_I0, W_t_W_I0_);

  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0);
  //  if (graphConfigPtr_->usingGnssFlag && usingFallbackGraphFlag_) {
  //    graphMgrPtr_->activateFallbackGraph();
  //  }
  // Read initial pose from graph
  T_W_I0 = gtsam::Pose3(graphMgrPtr_->getGraphState().navState().pose().matrix());
  std::cout << YELLOW_START << "GMsf " << GREEN_START << " INIT t(x,y,z): " << T_W_I0.translation().transpose()
            << ", RPY(deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Factor graph key of very first node: " << graphMgrPtr_->getStateKey() << std::endl;
  // Write in tf member variable
  T_W_I0_ = T_W_I0;
  // Initialize global pose
  T_W_Ik_ = T_W_I0_;
  imuTimeK_ = timeStamp_k;
  // Set flag
  initedGraphFlag_ = true;
}

void GraphMsf::optimizeGraph_() {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  // While loop
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Thread for updating graph is ready." << std::endl;
  while (true) {
    bool optimizeGraphFlag = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      if (optimizeGraphFlag_) {
        optimizeGraphFlag = optimizeGraphFlag_;
        optimizeGraphFlag_ = false;
      }
    }

    if (optimizeGraphFlag) {
      // Get result
      startLoopTime = std::chrono::high_resolution_clock::now();
      double currentTime;
      gtsam::NavState optimizedNavState = graphMgrPtr_->updateActiveGraphAndGetState(currentTime);
      endLoopTime = std::chrono::high_resolution_clock::now();

      if (graphConfigPtr_->verboseLevel > 0) {
        std::cout << YELLOW_START << "GMsf" << GREEN_START << " Whole optimization loop took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                  << COLOR_END << std::endl;
      }
      // Transform pose
      // TODO proper datatype
      T_W_I_opt_ = optimizedNavState.pose();
      optTime_ = currentTime;

      long seconds = long(currentTime);

    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

gtsam::Vector3 GraphMsf::W_t_W_Frame1_to_W_t_W_Frame2_(const gtsam::Point3& W_t_W_frame1, const std::string& frame1,
                                                       const std::string& frame2, const gtsam::Rot3& R_W_frame2) {
  // Static transforms
  const Eigen::Matrix4d& T_frame1_frame2 = staticTransformsPtr_->rv_T_frame1_frame2(frame1, frame2);
  const Eigen::Matrix4d& T_frame2_frame1 = staticTransformsPtr_->rv_T_frame1_frame2(frame2, frame1);
  Eigen::Vector3d frame1_t_frame1_frame2 = T_frame1_frame2.block<3, 1>(0, 3);

  /// Global rotation
  gtsam::Rot3 R_W_frame1 = R_W_frame2 * gtsam::Pose3(T_frame2_frame1).rotation();

  /// Translation in global frame
  Eigen::Vector3d W_t_frame1_frame2 = R_W_frame1 * frame1_t_frame1_frame2;

  /// Shift observed Gnss position to IMU frame (instead of Gnss antenna)
  return W_t_W_frame1 + W_t_frame1_frame2;
}

}  // namespace graph_msf
