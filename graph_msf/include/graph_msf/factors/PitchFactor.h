/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_PITCH_FACTOR_H
#define GRAPH_MSF_PITCH_FACTOR_H

#include <string>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>

namespace graph_msf {

/**
 * Factor to estimate rotation given gnss robot pitch
 */
class PitchFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured magnetometer reading, a 3-vector
   * @param model of the additive Gaussian noise that is assumed
   */
  PitchFactor(gtsam::Key j, double pitch, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), pitch_(pitch) {}

  // Destructor
  virtual ~PitchFactor() {}

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& q, boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const {
    // Jacobian
    if (H_Ptr) {
      (*H_Ptr) = (gtsam::Matrix(1, 6) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0).finished();  // [rad] [m]
    }

    // calculate error
    double pitch_error = q.rotation().pitch() - pitch_;

    while (pitch_error < -M_PI) pitch_error += 2 * M_PI;
    while (pitch_error > M_PI) pitch_error -= 2 * M_PI;

    return gtsam::Vector1(pitch_error);
  }

 private:
  double pitch_;  // pitch measurement
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_PITCH_FACTOR_H