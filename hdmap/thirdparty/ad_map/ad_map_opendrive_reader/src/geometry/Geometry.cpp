/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 * Copyright (C) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#include "opendrive/geometry/Geometry.h"

#include <boost/array.hpp>
#include <boost/math/tools/rational.hpp>
#include <cmath>
#include <stdexcept>

#include "opendrive/types.hpp"

namespace opendrive {
namespace geometry {

DirectedPoint::DirectedPoint() : location(0, 0), tangent(0) {}
DirectedPoint::DirectedPoint(const Point &point, double t)
    : location(point), tangent(t) {}
DirectedPoint::DirectedPoint(double x, double y, double t)
    : location(x, y), tangent(t) {}

void DirectedPoint::ApplyLateralOffset(double lateral_offset) {
  auto normal_x = -std::sin(tangent);
  auto normal_y = std::cos(tangent);
  location.x += lateral_offset * normal_x;
  location.y += lateral_offset * normal_y;
}

GeometryType Geometry::GetType() const { return _type; }
double Geometry::GetLength() const { return _length; }
double Geometry::GetStartOffset() const { return _start_position_offset; }
double Geometry::GetHeading() const { return _heading; }

const Point &Geometry::GetStartPosition() { return _start_position; }

Geometry::Geometry(GeometryType type, double start_offset, double length,
                   double heading, const Point &start_pos)
    : _type(type),
      _length(length),
      _start_position_offset(start_offset),
      _heading(heading),
      _start_position(start_pos) {
  if (_length == 0.) {
    throw std::invalid_argument("Geometry of length 0");
  }
}

GeometryLine::GeometryLine(double start_offset, double length, double heading,
                           const Point &start_pos)
    : Geometry(GeometryType::LINE, start_offset, length, heading, start_pos) {}

const DirectedPoint GeometryLine::PosFromDist(const double dist) const {
  DirectedPoint p(_start_position, _heading);
  p.location.x += dist * std::cos(p.tangent);
  p.location.y += dist * std::sin(p.tangent);
  return p;
}

GeometryArc::GeometryArc(double start_offset, double length, double heading,
                         const Point &start_pos, double curv)
    : Geometry(GeometryType::ARC, start_offset, length, heading, start_pos),
      _curvature(curv) {}

const DirectedPoint GeometryArc::PosFromDist(double dist) const {
  if (std::fabs(_curvature) < 1e-15) {
    // case not supported given the small curvature
    return DirectedPoint(_start_position, _heading);
  }
  const double radius = 1.0 / _curvature;
  const double theta = _heading - M_PI_2;
  double x = _start_position.x -
             radius * (cos(theta) - cos(theta + dist * _curvature));
  double y = _start_position.y -
             radius * (sin(theta) - sin(theta + dist * _curvature));

  double tangent = _heading + dist * _curvature;

  DirectedPoint p(x, y, tangent);

  return p;
}

double GeometryArc::GetCurvature() const { return _curvature; }

GeometryPoly3::GeometryPoly3(double start_offset, double length, double heading,
                             const Point &start_pos, double a, double b,
                             double c, double d)
    : Geometry(GeometryType::POLY3, start_offset, length, heading, start_pos),
      _a{a},
      _b{b},
      _c{c},
      _d{d} {}

const DirectedPoint GeometryPoly3::PosFromDist(const double dist) const {
  auto poly = boost::array<double, 4>{{_a, _b, _c, _d}};

  double u = dist;
  double v = boost::math::tools::evaluate_polynomial(poly, u);

  const double cos_t = std::cos(_heading);
  const double sin_t = std::sin(_heading);

  double x0 = _start_position.x;
  double y0 = _start_position.y;
  double x = u * cos_t - v * sin_t;
  double y = u * sin_t + v * cos_t;

  auto tangentPoly = boost::array<double, 4>{{_b, 2.0 * _c, 3.0 * _d}};

  double tangentV = boost::math::tools::evaluate_polynomial(tangentPoly, u);
  double theta = atan2(tangentV, 1.0);

  DirectedPoint point(x0 + x, y0 + y, _heading + theta);
  return point;
}

GeometryParamPoly3::GeometryParamPoly3(double start_offset, double length,
                                       double heading, const Point &start_pos,
                                       double aU, double bU, double cU,
                                       double dU, double aV, double bV,
                                       double cV, double dV)
    : Geometry(GeometryType::PARAMPOLY3, start_offset, length, heading,
               start_pos),
      _aU{aU},
      _bU{bU},
      _cU{cU},
      _dU{dU},
      _aV{aV},
      _bV{bV},
      _cV{cV},
      _dV{dV} {}

const DirectedPoint GeometryParamPoly3::PosFromDist(const double dist) const {
  double p = std::min(1.0, dist / _length);

  auto polyU = boost::array<double, 4>{{_aU, _bU, _cU, _dU}};
  auto polyV = boost::array<double, 4>{{_aV, _bV, _cV, _dV}};

  double u = boost::math::tools::evaluate_polynomial(polyU, p);
  double v = boost::math::tools::evaluate_polynomial(polyV, p);

  const double cos_t = std::cos(_heading);
  const double sin_t = std::sin(_heading);
  double x0 = _start_position.x;
  double y0 = _start_position.y;
  double x = u * cos_t - v * sin_t;
  double y = u * sin_t + v * cos_t;

  auto tangentPolyU = boost::array<double, 4>{{_bU, 2.0 * _cU, 3.0 * _dU, 0.0}};
  auto tangentPolyV = boost::array<double, 4>{{_bV, 2.0 * _cV, 3.0 * _dV, 0.0}};

  double tangentU = boost::math::tools::evaluate_polynomial(tangentPolyU, p);
  double tangentV = boost::math::tools::evaluate_polynomial(tangentPolyV, p);
  double theta = atan2(tangentV, tangentU);

  DirectedPoint point(x0 + x, y0 + y, _heading + theta);
  return point;
}

}  // namespace geometry
}  // namespace opendrive
