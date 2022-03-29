// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/serialize/ISerializer.hpp"
#include "ad/physics/Distance.hpp"
#include "ad/physics/Duration.hpp"
#include "ad/physics/MetricRange.hpp"
#include "ad/physics/ParametricRange.hpp"
#include "ad/physics/ParametricValue.hpp"
#include "ad/physics/Speed.hpp"
#include "ad/physics/Weight.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Serializer for physics::ParametricValue
 */
inline bool doSerialize(ISerializer &serializer, physics::ParametricValue &x)
{
  return serializer.serializeGeneratedType<physics::ParametricValue, double, SerializeableMagic::ParametricValue>(x);
}

/**
 * @brief Serializer for physics::Distance
 */
inline bool doSerialize(ISerializer &serializer, physics::Distance &x)
{
  return serializer.serializeGeneratedType<physics::Distance, double, SerializeableMagic::Distance>(x);
}

/**
 * @brief Serializer for physics::Duration
 */
inline bool doSerialize(ISerializer &serializer, physics::Duration &x)
{
  return serializer.serializeGeneratedType<physics::Duration, double, SerializeableMagic::Duration>(x);
}

/**
 * @brief Serializer for physics::Speed
 */
inline bool doSerialize(ISerializer &serializer, physics::Speed &x)
{
  return serializer.serializeGeneratedType<physics::Speed, double, SerializeableMagic::Speed>(x);
}

/**
 * @brief Serializer for physics::Weight
 */
inline bool doSerialize(ISerializer &serializer, physics::Weight &x)
{
  return serializer.serializeGeneratedType<physics::Weight, double, SerializeableMagic::Weight>(x);
}

/**
 * @brief Serializer for physics::MetricRange
 */
inline bool doSerialize(ISerializer &serializer, physics::MetricRange &x)
{
  return doSerialize(serializer, x.minimum) && doSerialize(serializer, x.maximum);
}

/**
 * @brief Serializer for physics::ParametricRange
 */
inline bool doSerialize(ISerializer &serializer, physics::ParametricRange &x)
{
  return serializer.serialize(SerializeableMagic::ParametricRange) && doSerialize(serializer, x.minimum)
    && doSerialize(serializer, x.maximum);
}

} // namespace serialize
} // namespace map
} // namespace ad
