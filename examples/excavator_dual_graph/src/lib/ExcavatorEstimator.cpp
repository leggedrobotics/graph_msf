/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_dual_graph/ExcavatorEstimator.h"

// Project
#include "excavator_dual_graph/ExcavatorStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/util/conversions.h"

namespace excavator_se {

ExcavatorEstimator::ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : privateNode_(*privateNodePtr) {
  std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<graph_msf::GraphConfig>();
  staticTransformsPtr_ = std::make_shared<ExcavatorStaticTransforms>(privateNodePtr);
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Get ROS params and set extrinsics
  readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

  if (not graph_msf::GraphMsfInterface::setup_()) {
    throw std::runtime_error("CompslamSeInterface could not be initiallized");
  }

  // Publishers ----------------------------
  initializePublishers_(privateNodePtr);

  // Subscribers ----------------------------
  initializeSubscribers_(privateNodePtr);

  // Messages ----------------------------
  initializeMessages_(privateNodePtr);

  std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void ExcavatorEstimator::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Odometry
  pubEstOdomImu_ = privateNode_.advertise<nav_msgs::Odometry>("/graph_msf/odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImu_ = privateNode_.advertise<nav_msgs::Odometry>("/graph_msf/odometry_map_imu", ROS_QUEUE_SIZE);
  // Paths
  pubEstOdomImuPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImuPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/est_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapGnssLPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_map_gnssL", ROS_QUEUE_SIZE);
  pubMeasMapGnssRPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_map_gnssR", ROS_QUEUE_SIZE);
  pubMeasMapLidarPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_map_Lidar", ROS_QUEUE_SIZE);
}

void ExcavatorEstimator::initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Imu
  subImu_ = privateNode_.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &ExcavatorEstimator::imuCallback_, this,
                                                     ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Initialized IMU cabin subscriber." << std::endl;

  // LiDAR Odometry
  subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &ExcavatorEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Initialized LiDAR Odometry subscriber." << std::endl;

  // GNSS
  if (graphConfigPtr_->usingGnssFlag) {
    subGnssLeft_.subscribe(*privateNodePtr, "/gnss_topic_1", ROS_QUEUE_SIZE);
    subGnssRight_.subscribe(*privateNodePtr, "/gnss_topic_2", ROS_QUEUE_SIZE);
    gnssExactSyncPtr_.reset(
        new message_filters::Synchronizer<GnssExactSyncPolicy>(GnssExactSyncPolicy(ROS_QUEUE_SIZE), subGnssLeft_, subGnssRight_));
    gnssExactSyncPtr_->registerCallback(boost::bind(&ExcavatorEstimator::gnssCallback_, this, _1, _2));
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Gnss subscriber (for both Gnss topics)." << std::endl;
  } else {
    std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START
              << " Gnss usage is set to false. Hence, lidar unary factors will be activated after graph initialization." << COLOR_END
              << std::endl;
  }
}

void ExcavatorEstimator::initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Odometry
  odomImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  mapImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  // Path
  estOdomImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  estMapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measMapLeftGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measMapRightGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measMapLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void ExcavatorEstimator::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  static bool firstCallbackFlag__ = true;
  if (firstCallbackFlag__) {
    firstCallbackFlag__ = false;
  }
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  graph_msf::GraphMsfInterface::addImuMeasurementAndPublishState_(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec());
}

void ExcavatorEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int odometryCallbackCounter__ = -1;
  static std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKm1Ptr__;

  // Counter
  ++odometryCallbackCounter__;

  Eigen::Matrix4d compslam_T_Wl_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, compslam_T_Wl_Lk);

  // Transform to IMU frame
  double timeK = odomLidarPtr->header.stamp.toSec();

  // Measurement
  std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKPtr;
  // Create pseudo unary factors
  if (graphConfigPtr_->usingGnssFlag) {
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    if (odometryCallbackCounter__ > 0) {
      graph_msf::GraphMsfInterface::addDualOdometryMeasurement_(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
    }
  } else {  // real unary factors
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    graph_msf::GraphMsfInterface::addUnaryPoseMeasurement_(*odometryKPtr);
  }

  if (!areYawAndPositionInited_() && (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15)) {
    std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START
              << " LiDAR odometry callback is setting global cabin yaw to 0, as it was not set so far." << COLOR_END << std::endl;
    graph_msf::GraphMsfInterface::initYawAndPosition_(
        compslam_T_Wl_Lk, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame());
  }

  // Wrap up iteration
  odometryKm1Ptr__ = odometryKPtr;

  // Add to path message
  addToPathMsg(measMapLidarPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               odomLidarPtr->header.stamp, compslam_T_Wl_Lk.block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasMapLidarPath_.publish(measMapLidarPathPtr_);
}

void ExcavatorEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                       const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedLeftCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d accumulatedRightCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d W_t_W_GnssL_km1__, W_t_W_GnssR_km1__;
  static int gnssCallbackCounter__ = 0;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d leftGnssCoord = Eigen::Vector3d(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);
  Eigen::Vector3d rightGnssCoord = Eigen::Vector3d(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude);
  Eigen::Vector3d estCovarianceXYZ(leftGnssMsgPtr->position_covariance[0], leftGnssMsgPtr->position_covariance[4],
                                   leftGnssMsgPtr->position_covariance[8]);
  if (!graphConfigPtr_->usingGnssFlag) {
    ROS_WARN("Received Gnss message, but usage is set to false.");
    graph_msf::GraphMsfInterface::activateFallbackGraph();
    return;
  } else if (gnssCallbackCounter__ < NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    // Wait until measurements got accumulated
    accumulatedLeftCoordinates__ += leftGnssCoord;
    accumulatedRightCoordinates__ += rightGnssCoord;
    if (!(gnssCallbackCounter__ % 10)) {
      std::cout << YELLOW_START << "ExcavatorEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    gnssHandlerPtr_->initHandler(accumulatedLeftCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START,
                                 accumulatedRightCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START);
  }

  // Convert to cartesian coordinates
  Eigen::Vector3d W_t_W_GnssL, W_t_W_GnssR;
  gnssHandlerPtr_->convertNavSatToPositions(leftGnssCoord, rightGnssCoord, W_t_W_GnssL, W_t_W_GnssR);
  double yaw_W_C = gnssHandlerPtr_->computeYaw(W_t_W_GnssL, W_t_W_GnssR);

  // Initialization
  if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    if (not graph_msf::GraphMsfInterface::initYawAndPosition_(
            yaw_W_C, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(), W_t_W_GnssL,
            dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame())) {
      // Decrease counter if not successfully initialized
      --gnssCallbackCounter__;
    }
  } else {
    graph_msf::UnaryMeasurement1D meas_yaw_W_C("Gnss yaw",
                                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(),
                                               int(gnssLeftRate_), leftGnssMsgPtr->header.stamp.toSec(), yaw_W_C, gnssHeadingUnaryNoise_);
    graph_msf::GraphMsfInterface::addGnssHeadingMeasurement_(meas_yaw_W_C);
    graph_msf::UnaryMeasurement3D meas_W_t_W_GnssL(
        "Gnss left", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame(), gnssLeftRate_,
        leftGnssMsgPtr->header.stamp.toSec(), W_t_W_GnssL,
        Eigen::Vector3d(gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_));
    graph_msf::GraphMsfInterface::addDualGnssPositionMeasurement_(meas_W_t_W_GnssL, W_t_W_GnssL_km1__, estCovarianceXYZ);
    graph_msf::UnaryMeasurement3D meas_W_t_W_GnssR(
        "Gnss right", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getRightGnssFrame(), gnssRightRate_,
        leftGnssMsgPtr->header.stamp.toSec(), W_t_W_GnssR,
        Eigen::Vector3d(gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_));
    graph_msf::GraphMsfInterface::addDualGnssPositionMeasurement_(meas_W_t_W_GnssR, W_t_W_GnssR_km1__, estCovarianceXYZ);
  }
  W_t_W_GnssL_km1__ = W_t_W_GnssL;
  W_t_W_GnssR_km1__ = W_t_W_GnssR;

  // Left GNSS
  /// Add to Path
  addToPathMsg(measMapLeftGnssPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               leftGnssMsgPtr->header.stamp, W_t_W_GnssL, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasMapGnssLPath_.publish(measMapLeftGnssPathPtr_);
  // Right GNSS
  /// Add to path
  addToPathMsg(measMapRightGnssPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               rightGnssMsgPtr->header.stamp, W_t_W_GnssR, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasMapGnssRPath_.publish(measMapRightGnssPathPtr_);
}

void ExcavatorEstimator::publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                       const Eigen::Vector3d& Ic_v_W_Ic, const Eigen::Vector3d& I_w_W_I) {
  // Used transforms
  Eigen::Matrix4d T_I_L = staticTransformsPtr_
                              ->rv_T_frame1_frame2(staticTransformsPtr_.get()->getImuFrame(),
                                                   dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame())
                              .inverse();

  // Pose
  Eigen::Matrix4d T_O_L = T_O_Ik * T_I_L;
  Eigen::Matrix4d T_W_L = T_W_O * T_O_L;

  // Publish odometry message for odom->imu with 100 Hz
  addToOdometryMsg(odomImuMsgPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(),
                   dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getImuFrame(), ros::Time(imuTimeK), T_O_Ik,
                   Ic_v_W_Ic, I_w_W_I);
  pubEstOdomImu_.publish(odomImuMsgPtr_);
  addToPathMsg(estOdomImuPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(),
               ros::Time(imuTimeK), T_O_Ik.block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 20);
  pubEstOdomImuPath_.publish(estOdomImuPathPtr_);
  // Publish odometry message for map->imu with 100 Hz
  addToOdometryMsg(mapImuMsgPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
                   dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getImuFrame(), ros::Time(imuTimeK), T_W_O * T_O_Ik,
                   Ic_v_W_Ic, I_w_W_I);
  pubEstOdomImu_.publish(mapImuMsgPtr_);
  addToPathMsg(estMapImuPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(), ros::Time(imuTimeK),
               (T_W_O * T_O_Ik).block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 20);
  pubEstMapImuPath_.publish(estMapImuPathPtr_);

  // Convert Measurements

  // Publish to TF
  // W_O
  static tf::Transform transform_W_O;
  transform_W_O.setOrigin(tf::Vector3(T_W_O(0, 3), T_W_O(1, 3), T_W_O(2, 3)));
  Eigen::Quaterniond q_W_O(T_W_O.block<3, 3>(0, 0));
  transform_W_O.setRotation(tf::Quaternion(q_W_O.x(), q_W_O.y(), q_W_O.z(), q_W_O.w()));
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_W_O, ros::Time(imuTimeK),
                                                    dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
                                                    dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame()));
  // I->B
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  listener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                            dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                            transform_I_B);

  // Publish T_O_B
  static tf::Transform transform_O_I;
  transform_O_I.setOrigin(tf::Vector3(T_O_Ik(0, 3), T_O_Ik(1, 3), T_O_Ik(2, 3)));
  Eigen::Quaterniond q_O_I(T_O_Ik.block<3, 3>(0, 0));
  transform_O_I.setRotation(tf::Quaternion(q_O_I.x(), q_O_I.y(), q_O_I.z(), q_O_I.w()));
  tfBroadcaster_.sendTransform(
      tf::StampedTransform(transform_O_I * transform_I_B, ros::Time(imuTimeK),
                           dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(),
                           dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame()));
}

}  // namespace excavator_se