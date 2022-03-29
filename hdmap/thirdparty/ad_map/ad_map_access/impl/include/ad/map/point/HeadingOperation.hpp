// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ECEFHeading.hpp"
#include "ad/map/point/ECEFPoint.hpp"
#include "ad/map/point/ENUHeading.hpp"
#include "ad/map/point/ENUPoint.hpp"
#include "ad/map/point/GeoPoint.hpp"
#include "ad/map/point/PointOperation.hpp"
#include "ad/physics/Angle.hpp"
#include "ad/physics/Distance.hpp"

namespace ad {
namespace map {
namespace point {

/**
 * @brief create a heading in ECEF as a directional vector
 *
 * @param[in] start  ECEFPoint defining the start of the directional vector
 * @param[in] end ECEFPoint defining the end of the directional vector
 *
 * @returns heading = norm(end - start)
 */
ECEFHeading createECEFHeading(ECEFPoint const &start, ECEFPoint const &end);

/**
 * @brief create a heading in ECEF as a directional vector
 *
 * @param[in] yaw ENUHeading the heading in ENU coordinate frame
 * @param[in] enuReferencePoint the GeoPoint defining the ENU reference point
 *
 * @returns heading in ECEF as directional vector
 */
ECEFHeading createECEFHeading(ENUHeading const &yaw,
                              GeoPoint const &enuReferencePoint);

/**
 * @brief create a ENUHeading from yaw angle in radians
 *
 * Heading in ENU coordinate system as angle measured from East to North axis
 * (yaw) in radians
 *
 * @param[in] yawAngleRadian heading provided as yaw angle in radians
 *
 * @returns heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 */
ENUHeading createENUHeading(double yawAngleRadian);

/**
 * @brief create a ENUHeading from angle
 *
 * Heading in ENU coordinate system as angle measured from East to North axis
 * (yaw) in radians
 *
 * @param[in] angle heading provided as yaw angle in radians
 *
 * @returns heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 */
ENUHeading createENUHeading(physics::Angle const &angle);

/**
 * @brief create a ENUHeading from ECEFHeading value
 *
 * Heading in ENU coordinate system as angle measured from East to North axis
 * (yaw) in radians
 *
 * @param[in] ecefHeading ECEFHeading value
 *
 * @returns heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 *
 */
ENUHeading createENUHeading(ECEFHeading const &ecefHeading);

/**
 * @brief create a ENUHeading from ECEFHeading value
 *
 * Heading in ENU coordinate system as angle measured from East to North axis
 * (yaw) in radians
 *
 * @param[in] ecefHeading ECEFHeading value
 * @param[in] enuReferencePoint the reference point of the ENU coordinate system
 * as GeoPoint
 *
 * @returns heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 *
 */
ENUHeading createENUHeading(ECEFHeading const &ecefHeading,
                            GeoPoint const &enuReferencePoint);

/**
 * @brief create a ENUHeading from ECEFHeading value
 *
 * Heading in ENU coordinate system as angle measured from East to North axis
 * (yaw) in radians
 *
 * @param[in] ecefHeading ECEFHeading value
 * @param[in] enuReferencePoint the reference point of the ENU coordinate system
 * as ECEFPoint
 *
 * @returns heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 *
 */
ENUHeading createENUHeading(ECEFHeading const &ecefHeading,
                            ECEFPoint const &enuReferencePoint);

/**
 * @brief create a ENUHeading from a directional vector
 *
 * @param[in] start point defining the start of the directional vector
 * @param[in] end point defining the end of the directional vector
 *
 * @returns ENU heading of the resulting directional vector
 */
ENUHeading createENUHeading(ENUPoint const &start, ENUPoint const &end);

/**
 * @returns normalized ENU heading
 *          heading with given yaw angle is normalized in the range -M_PI <
 * heading <= M_PI
 */
ENUHeading normalizeENUHeading(ENUHeading const &heading);

/**
 * @brief get a directional vector of the heading
 *
 * angle of zero vector will be along positive x - axis
 *
 * @param[in] heading the heading input
 */
inline ENUPoint getDirectionalVectorZPlane(ENUHeading const &heading) {
  ENUPoint directionalVector;
  directionalVector.x = ENUCoordinate(std::cos(static_cast<double>(heading)));
  directionalVector.y = ENUCoordinate(std::sin(static_cast<double>(heading)));
  directionalVector.z = ENUCoordinate(0.);
  return directionalVector;
}

/**
 * @brief get a vector that is orthogonal to the vectors heading
 *
 * angle of zero vector will be along positive y - axis
 *
 * @param[in] heading the heading input
 */
inline ENUPoint getOrthogonalVectorZPlane(ENUHeading const &heading) {
  ENUPoint orthogonalVector;
  orthogonalVector.x = ENUCoordinate(-std::sin(static_cast<double>(heading)));
  orthogonalVector.y = ENUCoordinate(std::cos(static_cast<double>(heading)));
  orthogonalVector.z = ENUCoordinate(0.);
  return orthogonalVector;
}

/**
 * @brief get a directional vector of the vectors heading and the orthogonal
 * vector at once
 *
 * Combines getDirectionalVectorZPlane() and getOrthogonalVectorZPlane() in one
 * call.
 *
 * @param[in] heading the heading input
 * @param[out] directionalVector directional vector of the heading,
 *    angle of zero vector will be along positive x - axis
 * @param[out] orthogonalVector vector that is orthogonal to the vectors heading
 *   angle of zero vector will be along positive y - axis
 */
inline void getDirectionVectorsZPlane(ENUHeading const &heading,
                                      ENUPoint &directionalVector,
                                      ENUPoint &orthogonalVector) {
  directionalVector = getDirectionalVectorZPlane(heading);
  orthogonalVector.x = -directionalVector.y;
  orthogonalVector.y = directionalVector.x;
  orthogonalVector.z = ENUCoordinate(0.);
}

}  // namespace point
}  // namespace map
}  // namespace ad

/**
 * @brief add two ECEFHeading vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns vector c = a + b
 */
inline ::ad::map::point::ECEFHeading operator+(
    ::ad::map::point::ECEFHeading const &a,
    ::ad::map::point::ECEFHeading const &b) {
  return ::ad::map::point::vectorAdd(a, b);
}

/**
 * @brief subtract two ECEFHeading vectors from each right
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns c = a - b
 */
inline ::ad::map::point::ECEFHeading operator-(
    ::ad::map::point::ECEFHeading const &a,
    ::ad::map::point::ECEFHeading const &b) {
  return ::ad::map::point::vectorSub(a, b);
}

/**
 * @brief calculate the dot product of two ECEFHeading vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns value d = a * b
 */
inline double operator*(::ad::map::point::ECEFHeading const &a,
                        ::ad::map::point::ECEFHeading const &b) {
  return ::ad::map::point::vectorDotProduct(a, b);
}

/**
 * @brief multiplies a ECEFHeading vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFHeading operator*(
    ::ad::map::point::ECEFHeading const &a, ::ad::physics::Distance const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFHeading vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ECEFHeading operator*(
    ::ad::physics::Distance const &b, ::ad::map::point::ECEFHeading const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFHeading vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFHeading operator*(
    ::ad::map::point::ECEFHeading const &a, double const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFHeading vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ECEFHeading operator*(
    double const &b, ::ad::map::point::ECEFHeading const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}
