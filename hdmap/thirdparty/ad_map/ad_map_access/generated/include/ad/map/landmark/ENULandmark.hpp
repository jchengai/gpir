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
#include "ad/map/landmark/LandmarkType.hpp"
#include "ad/map/landmark/TrafficLightType.hpp"
#include "ad/map/point/ENUHeading.hpp"
#include "ad/map/point/ENUPoint.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace landmark
 *
 * Handling of landmarks
 */
namespace landmark {

/*!
 * \brief DataType ENULandmark
 *
 * Landmark description in ENU coordiante frame.
 */
struct ENULandmark
{
  /*!
   * \brief Smart pointer on ENULandmark
   */
  typedef std::shared_ptr<ENULandmark> Ptr;

  /*!
   * \brief Smart pointer on constant ENULandmark
   */
  typedef std::shared_ptr<ENULandmark const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ENULandmark() = default;

  /*!
   * \brief standard destructor
   */
  ~ENULandmark() = default;

  /*!
   * \brief standard copy constructor
   */
  ENULandmark(const ENULandmark &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENULandmark(ENULandmark &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENULandmark
   *
   * \returns Reference to this ENULandmark.
   */
  ENULandmark &operator=(const ENULandmark &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENULandmark
   *
   * \returns Reference to this ENULandmark.
   */
  ENULandmark &operator=(ENULandmark &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENULandmark
   *
   * \returns \c true if both ENULandmark are equal
   */
  bool operator==(const ENULandmark &other) const
  {
    return (id == other.id) && (type == other.type) && (position == other.position) && (heading == other.heading)
      && (trafficLightType == other.trafficLightType);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENULandmark.
   *
   * \returns \c true if both ENULandmark are different
   */
  bool operator!=(const ENULandmark &other) const
  {
    return !operator==(other);
  }

  /*!
   * Identifier of the landmark.
   */
  ::ad::map::landmark::LandmarkId id{0};

  /*!
   * Type of the landmark.
   */
  ::ad::map::landmark::LandmarkType type{::ad::map::landmark::LandmarkType::INVALID};

  /*!
   * Position of the landmark
   */
  ::ad::map::point::ENUPoint position;

  /*!
   * Landmark 2D orientation regardind Z axis (A.K.A. yaw/heading) [rad]Directional heading of the landmark.
   */
  ::ad::map::point::ENUHeading heading;
  ::ad::map::landmark::TrafficLightType trafficLightType{::ad::map::landmark::TrafficLightType::INVALID};
};

} // namespace landmark
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANDMARK_ENULANDMARK
#define GEN_GUARD_AD_MAP_LANDMARK_ENULANDMARK
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace landmark
 *
 * Handling of landmarks
 */
namespace landmark {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value ENULandmark value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENULandmark const &_value)
{
  os << "ENULandmark(";
  os << "id:";
  os << _value.id;
  os << ",";
  os << "type:";
  os << _value.type;
  os << ",";
  os << "position:";
  os << _value.position;
  os << ",";
  os << "heading:";
  os << _value.heading;
  os << ",";
  os << "trafficLightType:";
  os << _value.trafficLightType;
  os << ")";
  return os;
}

} // namespace landmark
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ENULandmark
 */
inline std::string to_string(::ad::map::landmark::ENULandmark const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANDMARK_ENULANDMARK
