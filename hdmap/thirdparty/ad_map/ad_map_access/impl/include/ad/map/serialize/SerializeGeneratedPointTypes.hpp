// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/Altitude.hpp"
#include "ad/map/point/BoundingSphere.hpp"
#include "ad/map/point/ECEFCoordinate.hpp"
#include "ad/map/point/ECEFPoint.hpp"
#include "ad/map/point/ENUCoordinate.hpp"
#include "ad/map/point/ENUPoint.hpp"
#include "ad/map/point/GeoPoint.hpp"
#include "ad/map/point/Geometry.hpp"
#include "ad/map/point/Latitude.hpp"
#include "ad/map/point/Longitude.hpp"
#include "ad/map/point/ParaPoint.hpp"
#include "ad/map/serialize/ISerializer.hpp"
#include "ad/map/serialize/SerializeGeneratedBasicTypes.hpp"
#include "ad/map/serialize/SerializeGeneratedPhysicsTypes.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Serializer for point::ECEFCoordinate
 */
inline bool doSerialize(ISerializer &serializer, point::ECEFCoordinate &x)
{
  return serializer.serializeGeneratedType<point::ECEFCoordinate, double, SerializeableMagic::ECEFCoordinate>(x);
}

/**
 * @brief Serializer for point::ENUCoordinate
 */
inline bool doSerialize(ISerializer &serializer, point::ENUCoordinate &x)
{
  return serializer.serializeGeneratedType<point::ENUCoordinate, double, SerializeableMagic::ENUCoordinate>(x);
}

/**
 * @brief Serializer for point::Longitude
 */
inline bool doSerialize(ISerializer &serializer, point::Longitude &x)
{
  return serializer.serializeGeneratedType<point::Longitude, double, SerializeableMagic::Longitude>(x);
}

/**
 * @brief Serializer for point::Latitude
 */
inline bool doSerialize(ISerializer &serializer, point::Latitude &x)
{
  return serializer.serializeGeneratedType<point::Latitude, double, SerializeableMagic::Latitude>(x);
}

/**
 * @brief Serializer for point::Altitude
 */
inline bool doSerialize(ISerializer &serializer, point::Altitude &x)
{
  return serializer.serializeGeneratedType<point::Altitude, double, SerializeableMagic::Altitude>(x);
}

/**
 * @brief Serializer for point::ParaPoint
 */
inline bool doSerialize(ISerializer &serializer, point::ParaPoint &paraPoint)
{
  return serializer.serialize(SerializeableMagic::ParaPoint) && doSerialize(serializer, paraPoint.laneId)
    && doSerialize(serializer, paraPoint.parametricOffset);
}

/**
 * @brief Serializer for point::ENUPoint
 */
inline bool doSerialize(ISerializer &serializer, point::ENUPoint &enuPoint)
{
  //@todo: fix ENUCoordinate -> ENUPoint: currently here for backward compat
  return serializer.serialize(SerializeableMagic::ENUCoordinate) && doSerialize(serializer, enuPoint.x)
    && doSerialize(serializer, enuPoint.y) && doSerialize(serializer, enuPoint.z);
}

/**
 * @brief Serializer for point::ECEFPoint
 */
inline bool doSerialize(ISerializer &serializer, point::ECEFPoint &ecefPoint)
{
  //@todo: fix ECEFCoordinate -> ECEFPoint: currently here for backward compat
  return serializer.serialize(SerializeableMagic::ECEFCoordinate) && doSerialize(serializer, ecefPoint.x)
    && doSerialize(serializer, ecefPoint.y) && doSerialize(serializer, ecefPoint.z);
}

/**
 * @brief Serializer for point::GeoPoint
 */
inline bool doSerialize(ISerializer &serializer, point::GeoPoint &geoPoint)
{
  return serializer.serialize(SerializeableMagic::GeoPoint) && doSerialize(serializer, geoPoint.longitude)
    && doSerialize(serializer, geoPoint.latitude) && doSerialize(serializer, geoPoint.altitude);
}

/**
 * @brief Serializer for point::BoundingSphere
 */
inline bool doSerialize(ISerializer &serializer, point::BoundingSphere &boundingSphere)
{
  // @todo: keep original format, so no magic for bounding sphere for now
  return doSerialize(serializer, boundingSphere.center) && doSerialize(serializer, boundingSphere.radius);
}

/**
 * @brief Serializer for point::Geometry
 */
inline bool doSerialize(ISerializer &serializer, point::Geometry &geometry)
{
  if (!serializer.isStoring())
  {
    // clear cache when reading
    geometry.private_enuEdgeCache.enuVersion = 0;
    geometry.private_enuEdgeCache.enuEdge.clear();
  }
  bool ok = serializer.serialize(SerializeableMagic::Geometry) && serializer.serialize(geometry.isValid)
    && serializer.serialize(geometry.isClosed) && serialize::doSerialize(serializer, geometry.length);

  if (serializer.isStoring())
  {
    if (serializer.useEmbeddedPoints())
    {
      ok = ok && serializer.serializeObjectVector(geometry.ecefEdge);
    }
    else
    {
      static point::ECEFEdge empty;
      ok = ok && serializer.serializeObjectVector(empty);
    }
  }
  else
  {
    ok = ok && serializer.serializeObjectVector(geometry.ecefEdge);
  }
  return ok;
}

} // namespace serialize
} // namespace map
} // namespace ad
