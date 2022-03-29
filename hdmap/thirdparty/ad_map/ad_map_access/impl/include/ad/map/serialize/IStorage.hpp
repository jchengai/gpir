// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <string>

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Storage Interface
 */
class IStorage
{
protected: // Constructor/Destructor
  IStorage() = default;
  virtual ~IStorage() = default;

public: // To be implemented
  virtual const char *getStorageType() = 0;

protected: // To be implemented
  virtual bool doOpenForRead(std::string const &config) = 0;
  virtual bool doOpenForWrite(std::string const &config) = 0;
  virtual bool doCloseForRead() = 0;
  virtual bool doCloseForWrite() = 0;
  virtual bool doWrite(const void *x, std::size_t bytes) = 0;
  virtual bool doRead(void *x, std::size_t bytes) = 0;
};

} // namespace serialize
} // namespace map
} // namespace ad
