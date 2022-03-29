// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/serialize/Serializer.hpp"
#include "ad/map/access/Logging.hpp"

namespace ad {
namespace map {
namespace serialize {

////////////
// Constants

size_t Serializer::MAGIC = 0x03082018;
size_t Serializer::VERSION_MAJOR = 0;
size_t Serializer::VERSION_MINOR = 4;

/////////////////////////
// Constructor/Destructor

Serializer::Serializer(bool store, bool calc_checksum)
  : ISerializer(store)
{
  open_ = false;
  calc_checksum_ = calc_checksum;
}

Serializer::~Serializer()
{
}

/////////////
// Operations

bool Serializer::open(std::string const &config, size_t &version_major, size_t &version_minor)
{
  if (isStoring())
  {
    version_major = VERSION_MAJOR;
    version_minor = VERSION_MINOR;
    return openForWrite(config);
  }
  else
  {
    return openForRead(config, version_major, version_minor);
  }
}

bool Serializer::close()
{
  if (open_)
  {
    if (isStoring())
    {
      return closeForWrite();
    }
    else
    {
      return closeForRead();
    }
  }
  else
  {
    access::getLogger()->warn("Serializer: Attempt to close already closed stream.");
    return false;
  }
}

//////////////
// Aux Methods

bool Serializer::openForRead(std::string const &config, size_t &version_major, size_t &version_minor)
{
  if (doOpenForRead(config))
  {
    version_major = 0;
    version_minor = 0;
    size_t magic = 0;
    initChecksum();
    if (ISerializer::serialize(magic) && ISerializer::serialize(version_major) && ISerializer::serialize(version_minor))
    {
      if (magic == MAGIC)
      {
        if (version_major == VERSION_MAJOR && version_minor == VERSION_MINOR)
        {
          open_ = true;
          return true;
        }
        else
        {
          access::getLogger()->error(
            "Serializer: Version mismatch: Expected {}, Found {}", VERSION_MAJOR, version_major);
          throw std::runtime_error("Wrong map version!");
        }
      }
      else
      {
        access::getLogger()->error("Serializer: Invalid stream {}", config);
      }
    }
    else
    {
      access::getLogger()->error("Serializer: Invalid stream (too short) {}", config);
    }
    doCloseForRead();
  }
  else
  {
    access::getLogger()->error("Serializer: Cannot open stream for reading {}", config);
  }
  return false;
}

bool Serializer::openForWrite(std::string const &config)
{
  if (doOpenForWrite(config))
  {
    initChecksum();
    if (ISerializer::serialize(MAGIC) && ISerializer::serialize(VERSION_MAJOR) && ISerializer::serialize(VERSION_MINOR))
    {
      open_ = true;
      return true;
    }
    else
    {
      access::getLogger()->error("Serializer: Cannot write header to steam {}", config);
    }
    doCloseForWrite();
  }
  else
  {
    access::getLogger()->error("Serializer: Cannot open stream for writing {}", config);
  }
  return false;
}

bool Serializer::closeForRead()
{
  bool ret_val = false;
  if (!calc_checksum_ || checksumOK())
  {
    ret_val = true;
  }
  else
  {
    access::getLogger()->error("Serializer: Checksum mismatch!");
  }
  return doCloseForRead() && ret_val;
}

bool Serializer::closeForWrite()
{
  bool ret_val = false;
  if (!calc_checksum_ || writeChecksum())
  {
    ret_val = true;
  }
  else
  {
    access::getLogger()->error("Serializer: Unable to write checksum.");
  }
  return doCloseForWrite() && ret_val;
}

////////////////////
// Atomic Operations

bool Serializer::write(const void *x, size_t bytes)
{
  if (doWrite(x, bytes))
  {
    if (calc_checksum_)
    {
      updateChecksum(x, bytes);
    }
    return true;
  }
  else
  {
    access::getLogger()->error("Serializer: Unable to write {} bytes", bytes);
    return false;
  }
}

bool Serializer::read(void *x, size_t bytes)
{
  if (doRead(x, bytes))
  {
    if (calc_checksum_)
    {
      updateChecksum(x, bytes);
    }
    return true;
  }
  else
  {
    access::getLogger()->error("Serializer: Unable to read {} bytes", bytes);
    return false;
  }
}

} // namespace serialize
} // namespace map
} // namespace ad
