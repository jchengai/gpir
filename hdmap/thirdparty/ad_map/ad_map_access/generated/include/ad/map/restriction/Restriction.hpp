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
#include "ad/map/restriction/RoadUserTypeList.hpp"
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
 * \brief DataType Restriction
 */
struct Restriction {
  /*!
   * \brief Smart pointer on Restriction
   */
  typedef std::shared_ptr<Restriction> Ptr;

  /*!
   * \brief Smart pointer on constant Restriction
   */
  typedef std::shared_ptr<Restriction const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Restriction() = default;

  /*!
   * \brief standard destructor
   */
  ~Restriction() = default;

  /*!
   * \brief standard copy constructor
   */
  Restriction(const Restriction &other) = default;

  /*!
   * \brief standard move constructor
   */
  Restriction(Restriction &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Restriction
   *
   * \returns Reference to this Restriction.
   */
  Restriction &operator=(const Restriction &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Restriction
   *
   * \returns Reference to this Restriction.
   */
  Restriction &operator=(Restriction &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Restriction
   *
   * \returns \c true if both Restriction are equal
   */
  bool operator==(const Restriction &other) const {
    return (negated == other.negated) &&
           (roadUserTypes == other.roadUserTypes) &&
           (passengersMin == other.passengersMin);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Restriction.
   *
   * \returns \c true if both Restriction are different
   */
  bool operator!=(const Restriction &other) const { return !operator==(other); }

  bool negated{false};
  ::ad::map::restriction::RoadUserTypeList roadUserTypes;
  ::ad::map::restriction::PassengerCount passengersMin{0};
};

}  // namespace restriction
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTION
#define GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTION
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
 * \param[in] _value Restriction value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Restriction const &_value) {
  os << "Restriction(";
  os << "negated:";
  os << _value.negated;
  os << ",";
  os << "roadUserTypes:";
  os << _value.roadUserTypes;
  os << ",";
  os << "passengersMin:";
  os << _value.passengersMin;
  os << ")";
  return os;
}

}  // namespace restriction
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Restriction
 */
inline std::string to_string(::ad::map::restriction::Restriction const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTION
