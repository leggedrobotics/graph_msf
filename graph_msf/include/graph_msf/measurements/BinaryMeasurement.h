/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT_H
#define GRAPH_MSF_DELTAMEASUREMENT_H

#include "graph_msf/measurements/Measurement.h"

namespace graph_msf {

struct BinaryMeasurement : public Measurement {
 public:
  BinaryMeasurement(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeKm1,
                    const double timeK)
      : Measurement(measurementName, frameName, measurementRate, timeK), timeKm1_(timeKm1) {}

  virtual MeasurementType measurementType() override { return measurementType_; }

 protected:
  MeasurementType measurementType_ = MeasurementType::Binary;
  double timeKm1_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
