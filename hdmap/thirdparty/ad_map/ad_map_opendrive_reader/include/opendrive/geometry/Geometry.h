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

#pragma once

#include "opendrive/types.hpp"

namespace opendrive {
namespace geometry {

/**
 * @brief DirectedPoint is composed of a 2 dimensional position and an
 * orientation
 */
struct DirectedPoint {
  DirectedPoint();
  DirectedPoint(const Point &point, double t);
  DirectedPoint(double x, double y, double t);

  Point location = {0, 0};
  double tangent = 0.0;  // [radians]

  /**
   * @brief Shifts the point by lateral_offset meters in the normal direction of
   * the tangent
   */
  void ApplyLateralOffset(double lateral_offset);

  /**
   * @brief Compares two points, returns true if their parameters are identical
   */
  friend bool operator==(const DirectedPoint &lhs, const DirectedPoint &rhs) {
    return (lhs.location == rhs.location) && (lhs.tangent == rhs.tangent);
  }
};

class Geometry {
 public:
  /**
   * @brief Returns the GeometryType associated to this geometry.
   */
  GeometryType GetType() const;

  /**
   * @brief Returns the length of the geometry.
   */
  double GetLength() const;

  /**
   * @brief Returns the start offset in s coordinates.
   */
  double GetStartOffset() const;

  /**
   * @brief Returns the heading at the start position.
   */
  double GetHeading() const;

  /**
   * @brief Returns the start position in the global reference frame.
   */
  const Point &GetStartPosition();

  virtual ~Geometry() = default;

  /**
   * @brief Evaluates the geometry at a certain distance from the start
   * position.
   */
  virtual const DirectedPoint PosFromDist(double dist) const = 0;

 protected:
  Geometry(GeometryType type, double start_offset, double length,
           double heading, const Point &start_pos);

 protected:
  GeometryType _type;  // geometry type
  double _length;      // length of the road section [meters]

  double _start_position_offset;  // s-offset [meters]
  double _heading;                // start orientation [radians]

  Point _start_position;  // [meters]
};

class GeometryLine : public Geometry {
 public:
  GeometryLine(double start_offset, double length, double heading,
               const Point &start_pos);

  const DirectedPoint PosFromDist(const double dist) const override;
};

class GeometryArc : public Geometry {
 public:
  GeometryArc(double start_offset, double length, double heading,
              const Point &start_pos, double curv);

  const DirectedPoint PosFromDist(double dist) const override;

  double GetCurvature() const;

 private:
  double _curvature;
};

class GeometryPoly3 : public Geometry {
 public:
  GeometryPoly3(double start_offset, double length, double heading,
                const Point &start_pos, double a, double b, double c, double d);

  const DirectedPoint PosFromDist(const double dist) const override;

 private:
  double _a;
  double _b;
  double _c;
  double _d;
};

class GeometryParamPoly3 : public Geometry {
 public:
  GeometryParamPoly3(double start_offset, double length, double heading,
                     const Point &start_pos, double aU, double bU, double cU,
                     double dU, double aV, double bV, double cV, double dV);

  const DirectedPoint PosFromDist(const double dist) const override;

 private:
  double _aU;
  double _bU;
  double _cU;
  double _dU;
  double _aV;
  double _bV;
  double _cV;
  double _dV;
};

}  // namespace geometry
}  // namespace opendrive
