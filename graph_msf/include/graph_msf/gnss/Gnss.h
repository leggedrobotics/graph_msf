/*
Copyright 2017 by Dominic Jud, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GNSS_H
#define GNSS_H

#pragma once

#include <Eigen/Dense>

namespace graph_msf {

class Gnss {
 public:
  Gnss();
  virtual ~Gnss() = default;

  void setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude,
                    const double& referenceHeading);
  Eigen::Vector3d gpsToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude);
  Eigen::Vector3d cartesianToGps(const Eigen::Matrix<double, 3, 1> position);
  Eigen::Vector3d besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude);
  void setMercatorReferenceFrame(const Eigen::Vector3d newReferencePoint);

 protected:
  void calculateConversionParameters();

  /// Reference longitude of the GPS measurements [deg]
  double referenceLongitude_ = 7.43958;

  /// Reference latitude of the GPS measurements [deg]
  double referenceLatitude_ = 46.95241;

  /// Reference altitude of the GPS measurements [m]
  double referenceAltitude_ = 0.0;

  /// Reference heading for the GPS measurements [rad]
  double referenceHeading_ = 0.0;

  // Conversion parameters
  double earthRadiusN_ = 0.0;
  double earthRadiusE_ = 0.0;

  // WGS84 constants for conversion
  const double equatorialRadius_ = 6378137.0;
  const double flattening_ = 1.0 / 298.257223563;
  // calculate WGS84 constants
  const double excentricity2_ = 2 * flattening_ - flattening_ * flattening_;

  // As described in :
  // https://www.swisstopo.admin.ch/content/swisstopo-internet/de/topics/survey/reference-systems/switzerland/_jcr_content/contentPar/tabs/items/dokumente_publikatio/tabPar/downloadlist/downloadItems/517_1459343190376.download/refsys_d.pdf
  // Major Axis Bessel Ellipsoid
  const double a_ = 6377397.155;
  // 1. numerical Excentricity Bessel Ellipsoid
  const double e2_ = 0.006674372230614;
  // Latitude Bern
  const double phi0_ = 46.95241 * M_PI / 180.0;
  // Longitude Bern
  const double lambda0_ = 7.43958 * M_PI / 180.0;
  // Projection Sphere Radius
  const double r_ = a_ * sqrt(1 - e2_) / (1 - e2_ * sin(phi0_) * sin(phi0_));
  // Ratio Sphere Length to Ellipsoid Length;
  const double alpha_ = sqrt(1 + e2_ / (1 - e2_) * pow(cos(phi0_), 4));
  // Latitude of Zero Point on Sphere
  const double b0_ = asin(sin(phi0_) / alpha_);
  // Konstante der Breitenformel
  const double k_ = log(tan(M_PI_4 + b0_ / 2.0)) - alpha_ * log(tan(M_PI_4 + phi0_ / 2.0)) +
                    alpha_ * sqrt(e2_) / 2.0 * log((1 + sqrt(e2_) * sin(phi0_)) / (1 - sqrt(e2_) * sin(phi0_)));
  // xyz-offset
  Eigen::Vector3d xyzOffset_ = Eigen::Vector3d::Zero();
};

}  // namespace graph_msf

#endif  // GNSS_H
