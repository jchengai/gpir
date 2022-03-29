// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <string.h>

#include "ad/map/serialize/IStorage.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Interface for Checksum
 */
class IChecksum : virtual public IStorage
{
protected: // Constructor/Destructor
  IChecksum() = default;
  virtual ~IChecksum() = default;

public: // Info
  virtual const char *getChecksumType() = 0;

protected: // To be implemented: Checksum
  virtual void initChecksum() = 0;
  virtual void updateChecksum(const void *x, size_t bytes) = 0;
  virtual bool writeChecksum() = 0;
  virtual bool checksumOK() = 0;
};

} // namespace serialize
} // namespace map
} // namespace ad
