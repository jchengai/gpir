/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/**
 * Generated file
 * @file
 *
 * Generator Version : 11.0.0-1997
 */

#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include "ad/map/restriction/PassengerCount.hpp"
#include "ad/map/restriction/RoadUserType.hpp"
#include "ad/physics/Distance.hpp"
#include "ad/physics/Weight.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/*!
 * \brief DataType VehicleDescriptor
 */
struct VehicleDescriptor
{
  /*!
   * \brief Smart pointer on VehicleDescriptor
   */
  typedef std::shared_ptr<VehicleDescriptor> Ptr;

  /*!
   * \brief Smart pointer on constant VehicleDescriptor
   */
  typedef std::shared_ptr<VehicleDescriptor const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  VehicleDescriptor() = default;

  /*!
   * \brief standard destructor
   */
  ~VehicleDescriptor() = default;

  /*!
   * \brief standard copy constructor
   */
  VehicleDescriptor(const VehicleDescriptor &other) = default;

  /*!
   * \brief standard move constructor
   */
  VehicleDescriptor(VehicleDescriptor &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other VehicleDescriptor
   *
   * \returns Reference to this VehicleDescriptor.
   */
  VehicleDescriptor &operator=(const VehicleDescriptor &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other VehicleDescriptor
   *
   * \returns Reference to this VehicleDescriptor.
   */
  VehicleDescriptor &operator=(VehicleDescriptor &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other VehicleDescriptor
   *
   * \returns \c true if both VehicleDescriptor are equal
   */
  bool operator==(const VehicleDescriptor &other) const
  {
    return (passengers == other.passengers) && (type == other.type) && (width == other.width)
      && (height == other.height) && (length == other.length) && (weight == other.weight);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other VehicleDescriptor.
   *
   * \returns \c true if both VehicleDescriptor are different
   */
  bool operator!=(const VehicleDescriptor &other) const
  {
    return !operator==(other);
  }

  ::ad::map::restriction::PassengerCount passengers{0};
  ::ad::map::restriction::RoadUserType type{::ad::map::restriction::RoadUserType::INVALID};
  ::ad::physics::Distance width;
  ::ad::physics::Distance height;
  ::ad::physics::Distance length;
  ::ad::physics::Weight weight;
};

} // namespace restriction
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_RESTRICTION_VEHICLEDESCRIPTOR
#define GEN_GUARD_AD_MAP_RESTRICTION_VEHICLEDESCRIPTOR
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value VehicleDescriptor value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, VehicleDescriptor const &_value)
{
  os << "VehicleDescriptor(";
  os << "passengers:";
  os << _value.passengers;
  os << ",";
  os << "type:";
  os << _value.type;
  os << ",";
  os << "width:";
  os << _value.width;
  os << ",";
  os << "height:";
  os << _value.height;
  os << ",";
  os << "length:";
  os << _value.length;
  os << ",";
  os << "weight:";
  os << _value.weight;
  os << ")";
  return os;
}

} // namespace restriction
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for VehicleDescriptor
 */
inline std::string to_string(::ad::map::restriction::VehicleDescriptor const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_RESTRICTION_VEHICLEDESCRIPTOR
