/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT1D_H
#define GRAPH_MSF_UNARYMEASUREMENT1D_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

struct UnaryMeasurement1D : public UnaryMeasurement {
 public:
  UnaryMeasurement1D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const double measurement, const double measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurement_(measurement),
        measurementNoise_(measurementNoise) {}

  const double& measurementValue() const { return measurement_; }
  const double& measurementNoise() const { return measurementNoise_; }

 protected:
  double measurement_;
  double measurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
