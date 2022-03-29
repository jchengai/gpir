// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/serialize/StorageFile.hpp"
#include "ad/map/access/Logging.hpp"

namespace ad {
namespace map {
namespace serialize {

StorageFile::StorageFile()
{
  file_ = nullptr;
}

StorageFile::~StorageFile()
{
  if (file_ != nullptr)
  {
    access::getLogger()->error("StorageFile::dtor: File is not closed!");
    fclose(file_);
  }
}

//////////////////////////
// IStorage Implementation

bool StorageFile::doOpen(std::string const &config, std::string const &flags)
{
  if (file_ == nullptr)
  {
    file_ = fopen(config.c_str(), flags.c_str());
    return file_ != nullptr;
  }
  else
  {
    access::getLogger()->error("StorageFile::DoOpen: File already open! {}", config);
    return false;
  }
}

bool StorageFile::doClose()
{
  if (file_ != nullptr)
  {
    fclose(file_);
    file_ = nullptr;
    return true;
  }
  else
  {
    access::getLogger()->error("StorageFile: Attempt to close already closed file!");
    return false;
  }
}

bool StorageFile::doWrite(const void *x, size_t bytes)
{
  if (file_ != nullptr)
  {
    return fwrite(x, bytes, 1, file_) == 1;
  }
  else
  {
    access::getLogger()->error("StorageFile::DoWrite: File not open.");
    return false;
  }
}

bool StorageFile::doRead(void *x, size_t bytes)
{
  if (file_ != nullptr)
  {
    return fread(x, bytes, 1, file_) == 1;
  }
  else
  {
    access::getLogger()->error("StorageFile::DoRead: File not open.");
    return false;
  }
}

} // namespace serialize
} // namespace map
} // namespace ad
