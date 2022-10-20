/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT6D_H
#define GRAPH_MSF_UNARYMEASUREMENT6D_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

struct UnaryMeasurement6D : public UnaryMeasurement {
 public:
  UnaryMeasurement6D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const Eigen::Matrix4d& measurementPose, const Eigen::Matrix<double, 6, 1>& measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurementPose_(measurementPose),
        measurementNoise_(measurementNoise) {}

  const Eigen::Matrix4d& measurementPose() const { return measurementPose_; }
  const Eigen::Matrix<double, 6, 1>& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Matrix4d measurementPose_;
  Eigen::Matrix<double, 6, 1> measurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
