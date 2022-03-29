// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/serialize/ChecksumCRC32.hpp"
#include "ad/map/serialize/Serializer.hpp"
#include "ad/map/serialize/StorageFile.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Serializer for files with CRC32
 */
class SerializerFileCRC32 : virtual public Serializer, virtual public StorageFile, virtual public ChecksumCRC32
{
public: // Constructor/Destructor
  explicit SerializerFileCRC32(bool store)
    : Serializer(store, true)
  {
  }

  virtual ~SerializerFileCRC32() = default;
};

} // namespace serialize
} // namespace map
} // namespace ad
