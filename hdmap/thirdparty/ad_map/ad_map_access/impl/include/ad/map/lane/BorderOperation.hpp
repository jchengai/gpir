// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/Types.hpp"
#include "ad/map/point/ECEFOperation.hpp"
#include "ad/map/point/ENUOperation.hpp"
#include "ad/map/point/GeoOperation.hpp"
#include "ad/map/point/HeadingOperation.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace lane */
namespace lane {

/**
 * @brief Get the ENUEdge between the given border with corresponding
 * lateralAlignment
 *
 * @param[in] border the ENU border, the edge is calculated from
 * @param[in] lateralAlignment the lateral alignment as TParam [0.;1.] used to
 * calculate the resulting edge. The lateral alignment is relative to the left
 * edge. If lateralAlignment is 1., the left edge is returned, if
 * lateralAlignment is 0., the right edge is returned
 *
 * @throws std::invalid_argument if the lateralAlignment parameter is smaller
 * than 0. or larger than 1.
 */
point::ENUEdge getLateralAlignmentEdge(
    ENUBorder const &border, physics::ParametricValue const lateralAlignment);

/**
 * @brief Get the distance between an ENU point and the lateral alignment edge
 *
 * @param[in] enuPoint is the point for which the distance should be calculated
 * @param[in] lateralAlignmentEdge the lateral alignment in ENU form
 *
 * @return calculated Distance
 */
physics::Distance getDistanceEnuPointToLateralAlignmentEdge(
    point::ENUPoint const &enuPoint,
    point::ENUEdge const &lateralAlignmentEdge);

/**
 * @brief normalizes the border
 *
 * At first the left and right edges of the border are checked for irregular
 * directional vectors (scalarproduct of the vectors of three consecutive edge
 * points has to be positive) Irregular points are dropped.
 *
 * Then, the number of points of the left and right edge of the border are made
 * equal by extending the smaller edge. If an edge has less than 2 points,
 * nothing is done.
 *
 * The previousBorder (optional parameter) is used to extend the irregular
 * directional vector check to the beginning of the border edges; here leading
 * to an potential overwrite of the first edge point by the end of the
 * previousBorder.
 */
void normalizeBorder(ENUBorder &border,
                     ENUBorder const *previousBorder = nullptr);

/**
 * @brief operation to make the transition between two edges continuous
 *
 * If the end point of \c first is near (<=0.1m) to the end point of \c second
 * or one of the edges has less than two points, nothing is done. Otherwise the
 * first point of the second edge will exchanged by the last point of the first
 * edge. The second point of the second edge is placed at some distance along
 * the edge which is calculated using:
 * - the distance of the displacement of the two edges
 * - the direction of the displacement of the two edges compared to the the
 * corresponding direction of the edges
 *
 * @param[in] first the first edge (is untouched by the algorithm)
 * @param[in] second the second edge to be adapted if required to make the
 * transition continuous
 */
void makeTransitionToSecondEdgeContinuous(point::ENUEdge const &first,
                                          point::ENUEdge &second);

/**
 * @brief operation to make the transition between two borders continuous
 *
 * This executes the makeTransitionToSecondEdgeContinuous() for left and right
 * edges. In addition, adds interpolation points to the respective other edge if
 * required.
 *
 * @param[in] first the first border (is untouched by the algorithm)
 * @param[in] second the second border to be adapted if required to make the
 * transition continuous
 */
void makeTransitionToSecondBorderContinuous(ENUBorder const &first,
                                            ENUBorder &second);

/**
 * @brief operation to make the transition between two edges continuous
 *
 * If the end point of \c first is near (<=0.1m) to the end point of \c second
 * or one of the edges has less than two points, nothing is done. Otherwise the
 * first point of the second edge will exchanged by the last point of the first
 * edge. The second point of the second edge is placed at some distance along
 * the edge which is calculated using:
 * - the distance of the displacement of the two edges
 * - the direction of the displacement of the two edges compared to the the
 * corresponding direction of the edges
 *
 * @param[in] first the first edge to be adapted if required to make the
 * transition continuous
 * @param[in] second the second edge (is untouched by the algorithm)
 */
void makeTransitionFromFirstEdgeContinuous(point::ENUEdge &first,
                                           point::ENUEdge const &second);

/**
 * @brief operation to make the transition between two borders continuous
 *
 * This executes the makeTransitionToSecondEdgeContinuous() for left and right
 * edges. In addition, adds interpolation points to the respective other edge if
 * required.
 *
 * @param[in] first the first border to be adapted if required to make the
 * transition continuous
 * @param[in] second the second border (is untouched by the algorithm)
 */
void makeTransitionFromFirstBorderContinuous(ENUBorder &first,
                                             ENUBorder const &second);

/** @brief calculate the length of the provided border as distance value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within Cartesian ENU coordinate
 * frame.
 */
inline physics::Distance calcLength(ENUBorder const &border) {
  return (calcLength(border.left) + calcLength(border.right)) / 2.;
}
/** @brief calculate the length of the provided border as distance value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within Cartesian ECEF coordinate
 * frame.
 */
inline physics::Distance calcLength(ECEFBorder const &border) {
  return (calcLength(border.left) + calcLength(border.right)) / 2.;
}

/** @brief calculate the length of the provided border as distance value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within Cartesian ECEF coordinate
 * frame.
 */
inline physics::Distance calcLength(GeoBorder const &border) {
  return (calcLength(border.left) + calcLength(border.right)) / 2.;
}

/** @brief calculate the length out of the provided ENU border List as distance
 * value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within Cartesian ENU coordinate
 * frame.
 */
physics::Distance calcLength(ENUBorderList const &borderList);

/** @brief calculate the length out of the provided ECEF border List as distance
 * value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within Cartesian ECEF coordinate
 * frame.
 */
physics::Distance calcLength(ECEFBorderList const &borderList);

/** @brief calculate the length out of the provided GEO border List as distance
 * value
 *
 * For length calculation the average between left and right edge of the border
 * is returned. Length calculation is performed within GEO coordinate frame.
 */
physics::Distance calcLength(GeoBorderList const &borderList);

/** @brief calculate the ENUHeading of the vector<ENUBorder> at the given
 * ENUPoint
 *
 *  If the given ENUPoint is not within the given borders,
 *  an ENUHeading(2*M_PI) is returned.
 */
point::ENUHeading getENUHeading(ENUBorderList const &borderList,
                                point::ENUPoint const &enuPoint);

}  // namespace lane
}  // namespace map
}  // namespace ad
