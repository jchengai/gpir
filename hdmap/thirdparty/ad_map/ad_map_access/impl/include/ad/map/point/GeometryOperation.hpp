// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/CoordinateTransform.hpp"
#include "ad/map/point/ECEFEdge.hpp"
#include "ad/map/point/ENUEdge.hpp"
#include "ad/map/point/GeoEdge.hpp"
#include "ad/map/point/Geometry.hpp"
#include "ad/physics/ParametricRange.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace point */
namespace point {

/**
 * @brief validity check
 *
 * @param[in] geometry the geometry to check
 */
inline bool isValid(Geometry const &geometry) { return geometry.isValid; }

/**
 * @brief create a geometry
 * @param[in] points the points to create the geometry from
 * @param[in] closed should the geometry be closed?
 */
Geometry createGeometry(const ECEFEdge &points, bool closed);

/**
 * @brief get the cached ENUEdge for a geometry
 * @param[in] geometry the geometry to work on
 * @returns Polyline that defines this Geometry in the ENU frame.
 * \note    Prior to the method call, valid coordinate transformation object
 * must be set using SetCoordinateTransform(). ENU geometry will be calculated
 * on-the-fly if
 *          - it was not previously calculated, or
 *          - ENU reference point has been changed.
 */
ENUEdge getCachedENUEdge(Geometry const &geometry);

/**
 * @brief get the cached ENUEdge of center line
 * @param[in] left the left edge
 * @param[in] right the right edge
 */
bool calculateCachedENUCenterLine(Geometry const &left, Geometry const &right,
                                  ENUEdge &center);

/**
 * @brief Checks if Geometry is longitudinally connected with another Geometry
 * at the end.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Geometry longitudinally connected with another Geometry
 * at the end.
 */
bool isSuccessor(Geometry const &edge, const Geometry &other);

/**
 * @brief Checks if Geometry is longitudinally connected with another Geometry
 * at the start.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Geometry longitudinally connected with another Geometry
 * at the start.
 */
bool isPredecessor(Geometry const &edge, const Geometry &other);

/**
 * @brief Checks if two edges have same start point.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Geometry have same start point as another Geometry.
 */
bool haveSameStart(Geometry const &edge, const Geometry &other);

/**
 * @brief Checks if two edges have same end point.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Edge have same end point as another Edge.
 */
bool haveSameEnd(Geometry const &edge, const Geometry &other);

/**
 * @brief Calculates parametric point on the geometry.
 * @param[in] t Parameter. 0 will return first point, and 1 last point on the
 * geometry.
 * @return Parameteric point on the geometry. Can be invalid.
 */
point::ECEFPoint getParametricPoint(Geometry const &geometry,
                                    const physics::ParametricValue &t);

/**
 * @brief Generates sub-geometry for given range.
 * @param[in] geometry source geometry.
 * @param[in] trange Specifies parametric range.
 * @param[out] outputEdge The output edge to be filled with the sub-geometry
 * points
 * @param[in] revertOrder optional parameter: if set \c true the order of the
 * points in the outputEdge is in reverse order of the geometry
 * @return Sub-geometry.
 */
void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        ECEFEdge &outputEdge, const bool revertOrder = false);

/**
 * @brief Generates sub-geometry for given range.
 * @param[in] geometry source geometry.
 * @param[in] trange Specifies parametric range.
 * @param[out] outputEdge The output edge to be filled with the sub-geometry
 * points
 * @param[in] revertOrder optional parameter: if set \c true the order of the
 * points in the outputEdge is in reverse order of the geometry
 * @return Sub-geometry.
 */
void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        GeoEdge &outputEdge, const bool revertOrder = false);

/**
 * @brief Generates sub-geometry for given range.
 * This overloaded member internally makes use of the getCachedENUEdge() feature
 * of the geometry.
 *
 * @param[in] geometry source geometry.
 * @param[in] trange Specifies parametric range.
 * @param[out] outputEdge The output edge to be filled with the sub-geometry
 * points
 * @param[in] revertOrder optional parameter: if set \c true the order of the
 * points in the outputEdge is in reverse order of the geometry
 * @return Sub-geometry.
 */
void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        ENUEdge &outputEdge, const bool revertOrder = false);

/**
 * @brief Finds point on geometry nearest to given point.
 * @param[in] pt Point of interest.
 * @returns Parametric point on geometry nearest to the pt.
 *          Can be invalid (if pt is Invalid(), geometry is empty etc.).
 */
physics::ParametricValue findNearestPointOnEdge(Geometry const &geometry,
                                                const point::ECEFPoint &pt);

/**
 * @brief Calculates middle line between two Geometries.
 * @param[in] geometry A geometry
 * @param[in] other Another geometry.
 * @returns Middle line between two Geometry. Contains same number of points as
 * biggest one.
 */
ECEFEdge getMiddleEdge(Geometry const &geometry, Geometry const &other);

}  // namespace point
}  // namespace map
}  // namespace ad
