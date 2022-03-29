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

#include "ad/map/landmark/LandmarkId.hpp"
#include "ad/map/lane/ContactLocation.hpp"
#include "ad/map/lane/ContactTypeList.hpp"
#include "ad/map/lane/LaneId.hpp"
#include "ad/map/restriction/Restrictions.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/*!
 * \brief DataType ContactLane
 */
struct ContactLane {
  /*!
   * \brief Smart pointer on ContactLane
   */
  typedef std::shared_ptr<ContactLane> Ptr;

  /*!
   * \brief Smart pointer on constant ContactLane
   */
  typedef std::shared_ptr<ContactLane const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ContactLane() = default;

  /*!
   * \brief standard destructor
   */
  ~ContactLane() = default;

  /*!
   * \brief standard copy constructor
   */
  ContactLane(const ContactLane &other) = default;

  /*!
   * \brief standard move constructor
   */
  ContactLane(ContactLane &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ContactLane
   *
   * \returns Reference to this ContactLane.
   */
  ContactLane &operator=(const ContactLane &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ContactLane
   *
   * \returns Reference to this ContactLane.
   */
  ContactLane &operator=(ContactLane &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ContactLane
   *
   * \returns \c true if both ContactLane are equal
   */
  bool operator==(const ContactLane &other) const {
    return (toLane == other.toLane) && (location == other.location) &&
           (types == other.types) && (restrictions == other.restrictions) &&
           (trafficLightId == other.trafficLightId);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ContactLane.
   *
   * \returns \c true if both ContactLane are different
   */
  bool operator!=(const ContactLane &other) const { return !operator==(other); }

  /*!
   * Identifier of the lane to which connection is leading.
   */
  ::ad::map::lane::LaneId toLane{0};

  /*!
   * Contact Location
   */
  ::ad::map::lane::ContactLocation location{
      ::ad::map::lane::ContactLocation::INVALID};

  /*!
   * types of contact
   */
  ::ad::map::lane::ContactTypeList types;

  /*!
   * Contact restrictions
   */
  ::ad::map::restriction::Restrictions restrictions;

  /*!
   * Identifier of the traffic light belong to the contact
   * May be invalid if contact is not a traffic light
   */
  ::ad::map::landmark::LandmarkId trafficLightId{0};
};

}  // namespace lane
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_CONTACTLANE
#define GEN_GUARD_AD_MAP_LANE_CONTACTLANE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value ContactLane value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ContactLane const &_value) {
  os << "ContactLane(";
  os << "toLane:";
  os << _value.toLane;
  os << ",";
  os << "location:";
  os << _value.location;
  os << ",";
  os << "types:";
  os << _value.types;
  os << ",";
  os << "restrictions:";
  os << _value.restrictions;
  os << ",";
  os << "trafficLightId:";
  os << _value.trafficLightId;
  os << ")";
  return os;
}

}  // namespace lane
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ContactLane
 */
inline std::string to_string(::ad::map::lane::ContactLane const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_LANE_CONTACTLANE
