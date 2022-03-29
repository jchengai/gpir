// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "ad/map/serialize/SerializeableMagic.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace serialize */
namespace serialize {

/**
 * @brief Interface for Serializer
 */
class ISerializer
{
public: // Constructor/Destructor
  explicit ISerializer(bool store)
    : mIsStoring(store)
    , mUseMagic(true)
    , mUseEmbeddedPoints(true)
  {
  }

  virtual ~ISerializer() = default;

  bool isStoring() const
  {
    return mIsStoring;
  }

public: // Operations
  template <typename T> bool serialize(T &x)
  {
    if (mIsStoring)
    {
      return write(x);
    }
    else
    {
      return read(x);
    }
  }

  bool serialize(SerializeableMagic const &magic)
  {
    if (mUseMagic)
    {
      if (mIsStoring)
      {
        return write(static_cast<uint16_t>(magic));
      }
      else
      {
        uint16_t magicRead;
        auto const ok = read(magicRead) && (static_cast<uint16_t>(magic) == magicRead);
        return ok;
      }
    }
    else
    {
      return true;
    }
  }

  template <typename T, typename SerializeBaseType, SerializeableMagic magic> bool serializeGeneratedType(T &x)
  {
    if (serialize(magic))
    {
      if (mIsStoring)
      {
        return write(static_cast<SerializeBaseType>(x));
      }
      else
      {
        SerializeBaseType xD;
        if (read(xD))
        {
          x = T(xD);
          return true;
        }
        else
        {
          return false;
        }
      }
    }
    return false;
  }

  template <typename T> bool serializeVector(std::vector<T> &x)
  {
    if (mIsStoring)
    {
      return writeVector(x);
    }
    else
    {
      return readVector(x);
    }
  }

  template <typename T>
  bool serializeObjectVector(std::vector<T> &x, SerializeableMagic const &magic = SerializeableMagic::ObjectVectorType)
  {
    if (mIsStoring)
    {
      return writeObjectVector(x, magic);
    }
    else
    {
      return readObjectVector(x, magic);
    }
  }

  template <typename T, typename U, typename Comp> bool serializeObjectMap(std::map<T, U, Comp> &x)
  {
    if (mIsStoring)
    {
      return writeObjectMap(x);
    }
    else
    {
      return readObjectMap(x);
    }
  }

  template <typename T, typename U, typename Comp> bool serializeObjectPtrMap(std::map<T, std::shared_ptr<U>, Comp> &x)
  {
    if (mIsStoring)
    {
      return writeObjectPtrMap(x);
    }
    else
    {
      return readObjectPtrMap(x);
    }
  }

  template <typename T, typename U, typename Comp> bool serializeObjectVecMap(std::map<T, std::vector<U>, Comp> &x)
  {
    if (mIsStoring)
    {
      return writeObjectVecMap(x);
    }
    else
    {
      return readObjectVecMap(x);
    }
  }

private: // Operations
  template <typename T> bool read(T &x);
  template <typename T> bool read(T const &x);
  template <typename T> bool write(T const &x);

  template <typename T> bool readGeneratedDouble(T &x);
  template <typename T> bool writeGeneratedDouble(T const &x);

  template <typename T> bool readVector(std::vector<T> &x);
  template <typename T> bool writeVector(std::vector<T> &x);

  template <typename T>
  bool readObjectVector(std::vector<T> &x, SerializeableMagic const &magic = SerializeableMagic::ObjectVectorType);
  template <typename T>
  bool writeObjectVector(std::vector<T> &x, SerializeableMagic const &magic = SerializeableMagic::ObjectVectorType);

  template <typename T, typename U, typename Comp> bool readObjectMap(std::map<T, U, Comp> &x);
  template <typename T, typename U, typename Comp> bool writeObjectMap(std::map<T, U, Comp> &x);

  template <typename T, typename U, typename Comp> bool readObjectPtrMap(std::map<T, std::shared_ptr<U>, Comp> &x);
  template <typename T, typename U, typename Comp> bool writeObjectPtrMap(std::map<T, std::shared_ptr<U>, Comp> &x);

  template <typename T, typename U, typename Comp> bool readObjectVecMap(std::map<T, std::vector<U>, Comp> &x);
  template <typename T, typename U, typename Comp> bool writeObjectVecMap(std::map<T, std::vector<U>, Comp> &x);

public: // Aux Methods
  virtual bool write(const void *x, size_t bytes) = 0;
  virtual bool read(void *x, size_t bytes) = 0;

  /**
   * @brief Specifies if the every serialized block will be/is prefixed with object-specific
   *        magic number. This is application-wide setting.
   *
   * \warning Deserialization setting must use same value that is applied to serialization!
   *
   * @param[in] use_magic Set to true if every serialized block will be/is prefixed with
   *                      object-specific magic number.
   *
   * @return Old value of the flag.
   */
  bool setUseMagic(bool use_magic)
  {
    bool old_magic = mUseMagic;
    mUseMagic = use_magic;
    return old_magic;
  }

  /**
   * @brief Specifies if the geometry points should be embedded within the objects or handled separately.
   *
   * \warning Deserialization setting must use same value that is applied to serialization!
   *
   * @param[in] useEmbeddedPoints Set to true if every geometry object should embed the geometry points.
   *    If false, geometry points have to be handled separately.
   *
   * @return Old value of the flag.
   */
  bool setUseEmbeddedPoints(bool useEmbeddedPoints)
  {
    bool oldUseEmbeddedPoints = mUseEmbeddedPoints;
    mUseEmbeddedPoints = useEmbeddedPoints;
    return oldUseEmbeddedPoints;
  }

  /**
   * @brief retunrs the setting if embedded points should be used or not on serialization
   */
  bool useEmbeddedPoints() const
  {
    return mUseEmbeddedPoints;
  }

  /**
   * @Todo will delete this after preparing new map without connector
   */
  bool serializeEmptyObjectVec()
  {
    if (!serialize(SerializeableMagic::ObjectVectorMapType))
    {
      return false;
    }

    size_t n = 0;
    if (mIsStoring)
    {
      if (!write(n))
      {
        return false;
      }
    }
    else
    {
      if (!read(n))
      {
        return false;
      }
    }
    return true;
  }

private:                                 // Data Members
  bool mIsStoring;                       ///< If true, this is Serialization process, false means De-Serialization.
  bool mUseMagic;                        ///< If true, the SerializeableMagic is used within serialization
  bool mUseEmbeddedPoints;               ///< If true, the geometry points are saved together with objects
private:                                 // Special types
  typedef uint8_t EnumSerializationType; ///< Use 8 bits for enum serializations.
};

/////////////////
// Implementation

template <> inline bool ISerializer::write<bool>(bool const &x)
{
  uint8_t b = static_cast<uint8_t>(x ? 1 : 0);
  return write(b);
}

template <> inline bool ISerializer::read<bool>(bool &x)
{
  uint8_t b;
  if (read(b))
  {
    x = b != 0;
    return true;
  }
  else
  {
    return false;
  }
}

template <> inline bool ISerializer::write<std::string>(std::string const &value)
{
  uint32_t n = static_cast<uint32_t>(value.size());
  return serialize(n) && write(value.c_str(), n);
}

template <> inline bool ISerializer::read<std::string>(std::string &value)
{
  uint32_t n = 0;
  if (serialize(n))
  {
    char *buffer = new char[n + 1];
    if (buffer != nullptr)
    {
      if (read(buffer, n))
      {
        buffer[n] = '\0';
        value = buffer;
        delete[] buffer;
        return true;
      }
      delete[] buffer;
    }
  }
  return false;
}

inline bool doSerialize(ISerializer &serializer, std::string &value)
{
  return serializer.serialize(SerializeableMagic::String) && serializer.serialize(value);
}

template <typename T> inline bool ISerializer::write(T const &x)
{
#ifndef __GNUC__
  static_assert(std::is_trivially_copyable<T>::value, "Type needs non-trivial serialization!");
#endif
  if (std::is_enum<T>())
  {
    EnumSerializationType v = static_cast<EnumSerializationType>(x);
    return write(&v, sizeof(v));
  }
  else
  {
    return write(&x, sizeof(x));
  }
}

template <typename T> inline bool ISerializer::read(T &x)
{
#ifndef __GNUC__
  static_assert(std::is_trivially_copyable<T>::value, "Type needs non-trivial serialization!");
#endif
  if (std::is_enum<T>())
  {
    EnumSerializationType v;
    if (read(&v, sizeof(v)))
    {
      x = static_cast<T>(v);
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return read(&x, sizeof(x));
  }
}

template <typename T> inline bool ISerializer::writeVector(std::vector<T> &x)
{
  if (serialize(SerializeableMagic::VectorType))
  {
    size_t n = x.size();
    if (write(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        if (!write(x[i]))
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T> inline bool ISerializer::readVector(std::vector<T> &x)
{
  if (serialize(SerializeableMagic::VectorType))
  {
    size_t n;
    if (read(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        T xi;
        if (!read(xi))
        {
          return false;
        }
        x.push_back(xi);
      }
      return true;
    }
  }
  return false;
}

template <typename T> inline bool ISerializer::writeObjectVector(std::vector<T> &x, SerializeableMagic const &magic)
{
  if (serialize(magic))
  {
    size_t n = x.size();
    if (write(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        if (!doSerialize(*this, x[i]))
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T> inline bool ISerializer::readObjectVector(std::vector<T> &x, SerializeableMagic const &magic)
{
  if (serialize(magic))
  {
    size_t n;
    if (read(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        T xi;
        if (doSerialize(*this, xi))
        {
          x.push_back(xi);
        }
        else
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp> inline bool ISerializer::writeObjectMap(std::map<T, U, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectMapType))
  {
    size_t n = x.size();
    if (write(n))
    {
      typedef std::map<T, U> Map;
      for (typename Map::iterator it = x.begin(); it != x.end(); ++it)
      {
        T key = it->first;
        if (!doSerialize(*this, key) || !doSerialize(*this, it->second))
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp> inline bool ISerializer::readObjectMap(std::map<T, U, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectMapType))
  {
    size_t n;
    if (read(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        T first;
        if (doSerialize(*this, first))
        {
          U second;
          if (doSerialize(*this, second))
          {
            auto insertResult = x.insert({first, second});
            if (!insertResult.second)
            {
              return false;
            }
          }
          else
          {
            return false;
          }
        }
        else
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp>
inline bool ISerializer::writeObjectPtrMap(std::map<T, std::shared_ptr<U>, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectPtrMapType))
  {
    size_t n = x.size();
    if (write(n))
    {
      for (auto it = x.begin(); it != x.end(); it++)
      {
        T key = it->first;
        if (!doSerialize(*this, key) || !doSerialize(*this, *it->second))
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp>
inline bool ISerializer::readObjectPtrMap(std::map<T, std::shared_ptr<U>, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectPtrMapType))
  {
    size_t n;
    if (read(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        T first;
        if (doSerialize(*this, first))
        {
          std::shared_ptr<U> second = std::make_shared<U>();
          if (doSerialize(*this, *second))
          {
            auto insertResult = x.insert({first, second});
            if (!insertResult.second)
            {
              return false;
            }
          }
          else
          {
            return false;
          }
        }
        else
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp>
inline bool ISerializer::writeObjectVecMap(std::map<T, std::vector<U>, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectVectorMapType))
  {
    size_t n = x.size();
    if (write(n))
    {
      for (auto it = x.begin(); it != x.end(); it++)
      {
        T key = it->first;
        if (!doSerialize(*this, key) || !writeObjectVector(it->second))
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

template <typename T, typename U, typename Comp>
inline bool ISerializer::readObjectVecMap(std::map<T, std::vector<U>, Comp> &x)
{
  if (serialize(SerializeableMagic::ObjectVectorMapType))
  {
    size_t n;
    if (read(n))
    {
      for (size_t i = 0; i < n; i++)
      {
        T first;
        if (doSerialize(*this, first))
        {
          auto insertResult = x.insert({first, std::vector<U>()});
          if (!insertResult.second)
          {
            return false;
          }
          if (!readObjectVector(insertResult.first->second))
          {
            return false;
          }
        }
        else
        {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

} // namespace serialize
} // namespace map
} // namespace ad
