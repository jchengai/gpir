// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/LaneId.hpp"
#include "ad/map/serialize/ISerializer.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief serializer for lane::LaneId
 */
inline bool doSerialize(ISerializer &serializer, lane::LaneId &x)
{
  return serializer.serializeGeneratedType<lane::LaneId, uint64_t, SerializeableMagic::LaneId>(x);
}

} // namespace serialize
} // namespace map
} // namespace ad
