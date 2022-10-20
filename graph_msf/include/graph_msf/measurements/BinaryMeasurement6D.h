/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT6D_H
#define GRAPH_MSF_DELTAMEASUREMENT6D_H

#include "graph_msf/measurements/BinaryMeasurement.h"

namespace graph_msf {

struct BinaryMeasurement6D : public BinaryMeasurement {
 public:
  BinaryMeasurement6D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeKm1,
                      const double timeK, const Eigen::Matrix4d& T_delta, const Eigen::Matrix<double, 6, 1> measurementNoise)
      : BinaryMeasurement(measurementName, frameName, measurementRate, timeKm1, timeK),
        T_delta_(T_delta),
        measurementNoise_(measurementNoise) {}

  const Eigen::Matrix4d& T_delta() const { return T_delta_; }
  const Eigen::Matrix<double, 6, 1>& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Matrix4d T_delta_;
  Eigen::Matrix<double, 6, 1> measurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
