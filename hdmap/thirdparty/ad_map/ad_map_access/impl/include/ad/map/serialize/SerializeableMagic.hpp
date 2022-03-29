// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <cstdint>

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Declaration of class-specific Magic Numbers.
 */
enum class SerializeableMagic : uint16_t
{
  String = 0x0001,

  Base = 0x0818, ///< Basic Magic Number.

  Store = Base + 0,
  PassengerCount = Base + 1,
  ECEFCoordinate = Base + 2,
  ENUCoordinate = Base + 3,
  //  EgoCoordinate = Base + 4, <- not used, never implemented
  Distance = Base + 5,
  // Number = Base + 6, <- not used
  Speed = Base + 7,
  ParametricValue = Base + 8,
  ParametricRange = Base + 9,
  Weight = Base + 10,
  ComplianceVer = Base + 11,
  PartitionId = Base + 12,
  Duration = Base + 13,

  // AreaId = Base + 100,<- not used, never implemented
  LandmarkId = Base + 101,
  LaneId = Base + 102,
  // RoadId = Base + 103, <- not used, never implemented

  ECEFPoint = Base + 200,
  ENUPoint = Base + 201,
  // EgoPoint = Base + 202, -> not used, never implemented
  Geometry = Base + 203,
  Edge = Base + 204,
  ContactLane = Base + 205,
  Restriction = Base + 206,
  Restrictions = Base + 207,
  // VehicleDescriptor = Base + 208,  <- not required, we don't serialize vehicle descriptors
  // MapMatchedPosition = Base + 209, <- not required, we don't serialize map matched positions
  Lane = Base + 210,
  GeometryStore = Base + 211,
  GeometryStoreItem = Base + 212,
  // Connector = Base + 213,  <-not required, the connector is not used any more
  MapMetaData = Base + 214,

  GeoPoint = Base + 300,
  Longitude = Base + 301,
  Latitude = Base + 302,
  Altitude = Base + 303,

  SpeedLimit = Base + 400,
  // Para_Speeds = Base + 401, <- not used, as it's down to a vector
  ParaPoint = Base + 402,

  Landmark = Base + 500,

  // special magic values
  VectorType = 0xF016,
  ObjectVectorType = 0xF227,
  ObjectMapType = 0xF228,
  ObjectPtrMapType = 0xF229,
  ObjectPtrVectorType = 0xF337,
  ObjectVectorMapType = 0xF337,
};

} // namespace serialize
} // namespace map
} // namespace ad
