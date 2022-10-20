/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT3D_H
#define GRAPH_MSF_UNARYMEASUREMENT3D_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

struct UnaryMeasurement3D : public UnaryMeasurement {
 public:
  UnaryMeasurement3D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const Eigen::Vector3d& measurement, const Eigen::Vector3d& measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurement_(measurement),
        measurementNoise_(measurementNoise) {}

  const Eigen::Vector3d& measurementVector() const { return measurement_; }
  const Eigen::Vector3d& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Vector3d measurement_;
  Eigen::Vector3d measurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
