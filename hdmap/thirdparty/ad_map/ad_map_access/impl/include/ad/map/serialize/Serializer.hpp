// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <string>
#include <vector>

#include "ad/map/serialize/IChecksum.hpp"
#include "ad/map/serialize/ISerializer.hpp"
#include "ad/map/serialize/IStorage.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Serializer implementation
 */
class Serializer : public ISerializer, virtual public IChecksum
{
public: // Constants
  static size_t VERSION_MAJOR;
  static size_t VERSION_MINOR;

private: // Constants
  static size_t MAGIC;

public: // Constructor/Destructor
  Serializer(bool store, bool calc_checksum);
  virtual ~Serializer();

public: // Operations
  bool open(std::string const &config, size_t &version_major, size_t &version_minor);
  bool close();

private: // Aux Methods
  bool openForRead(std::string const &config, size_t &version_major, size_t &version_minor);
  bool openForWrite(std::string const &config);
  bool closeForRead();
  bool closeForWrite();

  bool write(const void *x, size_t bytes) override;
  bool read(void *x, size_t bytes) override;

private: // Data Members
  bool open_;
  bool calc_checksum_;
};

} // namespace serialize
} // namespace map
} // namespace ad
