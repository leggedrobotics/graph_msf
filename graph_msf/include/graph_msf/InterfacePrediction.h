/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef INTERFACE_PREDICTION_H
#define INTERFACE_PREDICTION_H

#include <Eigen/Eigen>

namespace graph_msf {

struct InterfacePrediction {
  InterfacePrediction(const Eigen::Matrix4d& T_W_O_, const Eigen::Matrix4d& T_O_Ik_, const Eigen::Vector3d& I_v_W_I_,
                      const Eigen::Vector3d& I_w_W_I_)
      : T_W_O(T_W_O_), T_O_Ik(T_O_Ik_), I_v_W_I(I_v_W_I_), I_w_W_I(I_w_W_I_) {}

  const Eigen::Matrix4d T_W_O;
  const Eigen::Matrix4d T_O_Ik;
  const Eigen::Vector3d I_v_W_I;
  const Eigen::Vector3d I_w_W_I;
};

}  // namespace graph_msf

#endif  // INTERFACE_PREDICTION_H
