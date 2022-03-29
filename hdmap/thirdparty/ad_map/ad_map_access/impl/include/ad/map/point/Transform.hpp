// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/access/Operation.hpp"
#include "ad/map/point/CoordinateTransform.hpp"
#include "ad/map/point/Types.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace point */
namespace point {

/**
 * @brief perform coordinate transformation from GeoPoint to ECEFPoint
 */
inline ECEFPoint toECEF(GeoPoint const &point) {
  return access::getCoordinateTransform()->Geo2ECEF(point);
}

/**
 * @brief perform coordinate transformation from ENUPoint to ECEFPoint
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ECEFPoint toECEF(ENUPoint const &point) {
  return access::getCoordinateTransform()->ENU2ECEF(point);
}

/**
 * @brief perform coordinate transformation from ENUPoint to ECEFPoint
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * enuReferencePoint
 */
ECEFPoint toECEF(ENUPoint const &point, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from ECEFPoint to GeoPoint
 */
inline GeoPoint toGeo(ECEFPoint const &point) {
  return access::getCoordinateTransform()->ECEF2Geo(point);
}

/**
 * @brief perform coordinate transformation from ENUPoint to GeoPoint
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline GeoPoint toGeo(ENUPoint const &point) {
  return access::getCoordinateTransform()->ENU2Geo(point);
}

/**
 * @brief perform coordinate transformation from ENUPoint to GeoPoint
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * enuReferencePoint
 */
GeoPoint toGeo(ENUPoint const &point, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from ECEFPoint to ENUPoint
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ENUPoint toENU(ECEFPoint const &point) {
  return access::getCoordinateTransform()->ECEF2ENU(point);
}

/**
 * @brief perform coordinate transformation from GeoPoint to ENUPoint
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ENUPoint toENU(GeoPoint const &point) {
  return access::getCoordinateTransform()->Geo2ENU(point);
}

/**
 * @brief perform coordinate transformation from ECEFPoint to ENUPoint
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * enuReferencePoint
 */
ENUPoint toENU(ECEFPoint const &point, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from GeoPoint to ENUPoint
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * enuReferencePoint
 */
ENUPoint toENU(GeoPoint const &point, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from GeoEdge to ECEFEdge
 */
inline ECEFEdge toECEF(GeoEdge const &edge) {
  ECEFEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from ENUEdge to ECEFEdge
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ECEFEdge toECEF(ENUEdge const &edge) {
  ECEFEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from ENUEdge to ECEFEdge
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * ENUReferencePoint
 */
ECEFEdge toECEF(ENUEdge const &edge, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from ECEFEdge to GeoEdge
 */
inline GeoEdge toGeo(ECEFEdge const &edge) {
  GeoEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from ENUEdge to GeoEdge
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline GeoEdge toGeo(ENUEdge const &edge) {
  GeoEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from ENUEdge to GeoEdge
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * ENUReferencePoint
 */
GeoEdge toGeo(ENUEdge const &edge, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from ECEFEdge to ENUEdge
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ENUEdge toENU(ECEFEdge const &edge) {
  ENUEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from GeoEdge to ENUEdge
 *
 * The transformation into ENU coordinate frame makes use of the globally set
 * ENUReferencePoint (see AdMapAccess::setENUReferencePoint())
 */
inline ENUEdge toENU(GeoEdge const &edge) {
  ENUEdge resultEdge;
  access::getCoordinateTransform()->convert(edge, resultEdge);
  return resultEdge;
}

/**
 * @brief perform coordinate transformation from ECEFEdge to ENUEdge
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * ENUReferencePoint
 */
ENUEdge toENU(ECEFEdge const &edge, GeoPoint const &enuReferencePoint);

/**
 * @brief perform coordinate transformation from GeoEdge to ENUEdge
 *
 * The transformation into ENU coordinate frame makes use of the provided
 * ENUReferencePoint
 */
ENUEdge toENU(GeoEdge const &edge, GeoPoint const &enuReferencePoint);

}  // namespace point
}  // namespace map
}  // namespace ad
