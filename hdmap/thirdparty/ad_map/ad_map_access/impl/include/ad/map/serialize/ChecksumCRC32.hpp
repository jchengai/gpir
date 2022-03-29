// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <cstdint>

#include "ad/map/serialize/IChecksum.hpp"
#include "ad/map/serialize/IStorage.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Standalone checksum CRC32 calculation
 */
uint32_t calculateCRC32Checksum(const void *x, size_t bytes);

/**
 * @brief Checksum CRC32 calculation implementation
 */
class ChecksumCRC32 : virtual public IChecksum, virtual public IStorage
{
protected: // Constructor/Destructor
  ChecksumCRC32();
  virtual ~ChecksumCRC32();

public: // Overriden IChecksum
  const char *getChecksumType() override
  {
    return "CRC-32";
  }

protected: // Overriden IChecksum
  void initChecksum() override;
  void updateChecksum(const void *x, size_t bytes) override;
  bool writeChecksum() override;
  bool checksumOK() override;

private:
  uint32_t mCrc;
};

} // namespace serialize
} // namespace map
} // namespace ad
