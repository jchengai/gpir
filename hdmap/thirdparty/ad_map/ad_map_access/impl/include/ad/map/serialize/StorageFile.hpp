// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <stdio.h>
#include "ad/map/serialize/IStorage.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief File Storage implementation
 */
class StorageFile : protected virtual IStorage
{
protected: // Constructor/Destructor
  StorageFile();
  virtual ~StorageFile();

public: // IStorage Implementation
  const char *getStorageType() override
  {
    return "File";
  }

protected: // IStorage Implementation
  bool doOpenForRead(std::string const &config) override
  {
    return doOpen(config, "rb");
  };

  bool doOpenForWrite(std::string const &config) override
  {
    return doOpen(config, "wb");
  }

  bool doCloseForRead() override
  {
    return doClose();
  }

  bool doCloseForWrite() override
  {
    return doClose();
  }

  bool doWrite(const void *x, size_t bytes) override;
  bool doRead(void *x, size_t bytes) override;

private: // Aux Methods
  bool doOpen(std::string const &config, std::string const &flags);
  bool doClose();

private: // Data Members
  FILE *file_;
};

} // namespace serialize
} // namespace map
} // namespace ad
