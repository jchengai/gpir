// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/point/Transform.hpp"

#include "ad/map/access/Operation.hpp"
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace point {

ECEFPoint toECEF(ENUPoint const &point, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  return coordinateTransform.ENU2ECEF(point);
}

GeoPoint toGeo(ENUPoint const &point, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  return coordinateTransform.ENU2Geo(point);
}

ENUPoint toENU(ECEFPoint const &point, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  return coordinateTransform.ECEF2ENU(point);
}

ENUPoint toENU(GeoPoint const &point, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  return coordinateTransform.Geo2ENU(point);
}

ECEFEdge toECEF(ENUEdge const &edge, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  ECEFEdge resultEdge;
  coordinateTransform.convert(edge, resultEdge);
  return resultEdge;
}

GeoEdge toGeo(ENUEdge const &edge, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  GeoEdge resultEdge;
  coordinateTransform.convert(edge, resultEdge);
  return resultEdge;
}

ENUEdge toENU(ECEFEdge const &edge, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  ENUEdge resultEdge;
  coordinateTransform.convert(edge, resultEdge);
  return resultEdge;
}

ENUEdge toENU(GeoEdge const &edge, GeoPoint const &enuReferencePoint)
{
  CoordinateTransform coordinateTransform;
  coordinateTransform.setENUReferencePoint(enuReferencePoint);
  ENUEdge resultEdge;
  coordinateTransform.convert(edge, resultEdge);
  return resultEdge;
}

} // namespace point
} // namespace map
} // namespace ad
