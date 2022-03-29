// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/GeoEdgeValidInputRange.hpp"
#include "ad/map/point/GeoPointValidInputRange.hpp"
#include "ad/map/point/PointOperation.hpp"
#include "ad/physics/Distance.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace point */
namespace point {

/**
 * @brief constant defining the unknown altitude
 */
extern Altitude const AltitudeUnknown;

/**
 * @brief checks if the given GeoPoint is valid
 *
 * The point is valid if it's within valid input range.
 */
inline bool isValid(GeoPoint const &point, bool const logErrors = true) {
  return withinValidInputRange(point, logErrors);
}

/**
 * @brief checks if the given GeoEdge is valid
 *
 * The point is valid if it's within valid input range.
 */
inline bool isValid(GeoEdge const &edge, bool const logErrors = true) {
  return withinValidInputRange(edge, logErrors);
}

/**
 * @brief Convert degrees to radians.
 * @param[in] deg Decimal degrees.
 * @return Radians.
 */
inline double degree2radians(double degree) { return degree * M_PI / 180.0; }

/**
 * @brief Convert radians to degrees.
 * @param[in]  rad Radians.
 * @return Degrees.
 */
inline double radians2degree(double radians) { return radians * 180.0 / M_PI; }

/**
 * @brief Convert Latitude to radians.
 * @param[in] latitude latitude
 * @return latitude in radians.
 */
inline double toRadians(Latitude const &latitude) {
  return degree2radians(static_cast<double>(latitude));
}

/**
 * @brief Convert Longitude to radians.
 * @param[in] longitude longitude
 * @return longitude in radians.
 */
inline double toRadians(Longitude const &longitude) {
  return degree2radians(static_cast<double>(longitude));
}

/**
 * @brief create a GeoPoint
 *
 * @param[in] longitude longitude of the point
 * @param[in] latitude latitude of the point
 * @param[in] altitude altitude of the point
 */
inline GeoPoint createGeoPoint(Longitude const longitude,
                               Latitude const latitude,
                               Altitude const altitude) {
  GeoPoint result;
  result.longitude = longitude;
  result.latitude = latitude;
  result.altitude = altitude;
  return result;
}

/**
 * @brief Computes distance between geo points.
 * @returns Distance between two points in meters.
 */
physics::Distance distance(GeoPoint const &point, GeoPoint const &other);

/**
 * @returns New point with same longitude and latitude, but zero altitude.
 */
GeoPoint zeroAltitude(GeoPoint const &point);

/**
 * @brief Calculated distance between points not taking in account altitude.
 * @param[in] other Other object. Must be IsValid()!
 * @returns Distance between this and other point not taking in account
 * altitude.
 */
physics::Distance flatDistance(GeoPoint const &point, const GeoPoint &other);

/**
 * @brief Approximates altitude of the point based on other points.
 * @param[in] pts Set of other points that will be used in approximation.
 * @returns New point with same longitude and latitude, but approximated
 * altitude.
 */
GeoPoint approxAltitude(GeoPoint const &point, const GeoEdge &pts);

/**
 * @brief Checks if point is on the left side of the line defined by two points.
 * @param[in] pt0 First point defining the line.
 * @param[in] pt1 Second point defining the line.
 * @returns true if this point is on the left of the line defined by pt0, pt1.
 * \note Altitude is not taken in the account!
 */
bool isOnTheLeft(GeoPoint const &point, const GeoPoint &pt0,
                 const GeoPoint &pt1);

/**
 * @brief Checks if two GeoEdge have same orientation by taking
 *        in account distances between first and last points.
 * @param[in] pts0 First GeoEdge
 * @param[in] pts1 Second GeoEdge
 * @returns true if d(pt0[0], pt1[0]) <= d(pt0[0], pt1[last])
 */
bool haveSameOrientation(const GeoEdge &pts0, const GeoEdge &pts1);

/**
 * @brief Checks if one polyline is on the left side of another polyline.
 * @param[in] pts0 First GeoEdge
 * @param[in] pts1 Second GeoEdge
 * @returns true if pts0 is left of pts1.
 * \note Altitude is not taken in the account!
 */
bool isOnTheLeft(const GeoEdge &pts0, const GeoEdge &pts1);

/** @brief calculate the length of the provided border as distance value
 */
physics::Distance calcLength(GeoEdge const &edge);

/** @brief specialization of vectorExtrapolate for GeoPoint
 */
template <>
inline GeoPoint vectorExtrapolate(GeoPoint const &a, GeoPoint const &b,
                                  double const &scalar) {
  GeoPoint result;
  result.longitude = (1 - scalar) * a.longitude + scalar * b.longitude;
  result.latitude = (1 - scalar) * a.latitude + scalar * b.latitude;
  result.altitude = (1 - scalar) * a.altitude + scalar * b.altitude;
  return result;
}

}  // namespace point
}  // namespace map
}  // namespace ad
