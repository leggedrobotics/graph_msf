/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ELEMENTTOROOT_H
#define ELEMENTTOROOT_H

#include "tf/tf.h"

namespace graph_msf {

class ElementToRoot final {
 public:
  /// Constructor
  explicit ElementToRoot(const tf::Transform& T, const std::string& rootName_, const std::string& elementName_)
      : T_root_element(T), rootName(rootName_), elementName(elementName_) {}

  tf::Transform T_root_element;  ///< The KDL segment
  std::string rootName;          ///< The name of the root element to which this link is attached
  std::string elementName;       ///< The name of the element
};

}  // namespace graph_msf

#endif  // ELEMENTTOROOT_H
