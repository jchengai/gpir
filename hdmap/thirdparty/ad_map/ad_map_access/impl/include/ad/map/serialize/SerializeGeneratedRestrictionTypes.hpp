// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/restriction/Types.hpp"
#include "ad/map/serialize/ISerializer.hpp"
#include "ad/map/serialize/SerializeGeneratedPhysicsTypes.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief serializer for restriction::PassengerCount
 */
inline bool doSerialize(ISerializer &serializer, restriction::PassengerCount &x)
{
  return serializer.serializeGeneratedType<restriction::PassengerCount, uint16_t, SerializeableMagic::PassengerCount>(
    x);
}

/**
 * @brief serializer for restriction::Restriction
 */
inline bool doSerialize(ISerializer &serializer, restriction::Restriction &x)
{
  return serializer.serialize(SerializeableMagic::Restriction) && serializer.serialize(x.negated)
    && doSerialize(serializer, x.passengersMin) && serializer.serializeVector(x.roadUserTypes);
}

/**
 * @brief serializer for restriction::Restrictions
 */
inline bool doSerialize(ISerializer &serializer, restriction::Restrictions &x)
{
  return serializer.serialize(SerializeableMagic::Restrictions)
    //@todo: special serializable magic for backward compatibility
    && serializer.serializeObjectVector(x.conjunctions, SerializeableMagic::ObjectPtrVectorType)
    && serializer.serializeObjectVector(x.disjunctions, SerializeableMagic::ObjectPtrVectorType);
}

/**
 * @brief serializer for restriction::SpeedLimit
 */
inline bool doSerialize(ISerializer &serializer, restriction::SpeedLimit &x)
{
  bool ok = serializer.serialize(SerializeableMagic::SpeedLimit) && doSerialize(serializer, x.speedLimit)
    && doSerialize(serializer, x.lanePiece);

  if (ok)
  {
    // speed limits have to be actually positive!
    if (x.speedLimit <= physics::Speed(0.))
    {
      x.speedLimit = std::numeric_limits<physics::Speed>::max();
    }
  }
  return ok;
}

} // namespace serialize
} // namespace map
} // namespace ad
