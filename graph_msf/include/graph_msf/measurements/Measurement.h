/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_MEASUREMENT_H
#define GRAPH_MSF_MEASUREMENT_H

#include <Eigen/Eigen>
#include <string>

namespace graph_msf {

// Enum that contains 2 possible measurement types
enum class MeasurementType { Unary, Binary };

struct Measurement {
 public:
  Measurement(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp)
      : measurementName_(measurementName), frameName_(frameName), measurementRate_(measurementRate), timeK_(timeStamp) {}

  // Public Methods
  const std::string& measurementName() const { return measurementName_; }
  const std::string& frameName() const { return frameName_; }
  int measurementRate() const { return measurementRate_; }
  double timeK() const { return timeK_; }

  // Pure Virtual Class
  virtual MeasurementType measurementType() = 0;

 protected:
  // Members
  std::string measurementName_;
  std::string frameName_;
  int measurementRate_;
  double timeK_;

  // Unknown at this level
  MeasurementType measurementType_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_MEASUREMENT_H
