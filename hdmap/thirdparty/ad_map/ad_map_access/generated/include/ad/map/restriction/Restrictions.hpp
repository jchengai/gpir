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
#include <memory>
#include <sstream>

#include "ad/map/restriction/RestrictionList.hpp"
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
 * \brief DataType Restrictions
 */
struct Restrictions {
  /*!
   * \brief Smart pointer on Restrictions
   */
  typedef std::shared_ptr<Restrictions> Ptr;

  /*!
   * \brief Smart pointer on constant Restrictions
   */
  typedef std::shared_ptr<Restrictions const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Restrictions() = default;

  /*!
   * \brief standard destructor
   */
  ~Restrictions() = default;

  /*!
   * \brief standard copy constructor
   */
  Restrictions(const Restrictions &other) = default;

  /*!
   * \brief standard move constructor
   */
  Restrictions(Restrictions &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Restrictions
   *
   * \returns Reference to this Restrictions.
   */
  Restrictions &operator=(const Restrictions &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Restrictions
   *
   * \returns Reference to this Restrictions.
   */
  Restrictions &operator=(Restrictions &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Restrictions
   *
   * \returns \c true if both Restrictions are equal
   */
  bool operator==(const Restrictions &other) const {
    return (conjunctions == other.conjunctions) &&
           (disjunctions == other.disjunctions);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Restrictions.
   *
   * \returns \c true if both Restrictions are different
   */
  bool operator!=(const Restrictions &other) const {
    return !operator==(other);
  }

  ::ad::map::restriction::RestrictionList conjunctions;
  ::ad::map::restriction::RestrictionList disjunctions;
};

}  // namespace restriction
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTIONS
#define GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTIONS
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
 * \param[in] _value Restrictions value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Restrictions const &_value) {
  os << "Restrictions(";
  os << "conjunctions:";
  os << _value.conjunctions;
  os << ",";
  os << "disjunctions:";
  os << _value.disjunctions;
  os << ")";
  return os;
}

}  // namespace restriction
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Restrictions
 */
inline std::string to_string(
    ::ad::map::restriction::Restrictions const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_RESTRICTION_RESTRICTIONS
