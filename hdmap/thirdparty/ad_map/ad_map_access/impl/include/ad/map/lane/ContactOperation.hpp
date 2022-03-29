// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/ContactLaneValidInputRange.hpp"
#include "ad/map/lane/Types.hpp"
#include "ad/map/restriction/RestrictionOperation.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace lane */
namespace lane {

/**
 * @brief Provides opposite Location.
 * @param[in] e Base location.
 * @returns Opposite location. INVALID if there is no opposite.
 */
inline ContactLocation oppositeLocation(ContactLocation const &e) {
  switch (e) {
    case ContactLocation::INVALID:
      return ContactLocation::INVALID;
    case ContactLocation::UNKNOWN:
      return ContactLocation::INVALID;
    case ContactLocation::LEFT:
      return ContactLocation::RIGHT;
    case ContactLocation::RIGHT:
      return ContactLocation::LEFT;
    case ContactLocation::SUCCESSOR:
      return ContactLocation::PREDECESSOR;
    case ContactLocation::PREDECESSOR:
      return ContactLocation::SUCCESSOR;
    case ContactLocation::OVERLAP:
      return ContactLocation::OVERLAP;
    default:
      return ContactLocation::INVALID;
  }
}

/**
 * @brief Checks if vehicle fits the restriction criteria of the contact lane.
 * @param[in] contactLane contact lane.
 * @param[in] vehicle Description of the vehicle.
 * @returns true of vehicle fits the restrictions criteria.
 */
inline bool isAccessOk(ContactLane const &contactLane,
                       restriction::VehicleDescriptor const &vehicle) {
  return restriction::isAccessOk(contactLane.restrictions, vehicle);
}

/**
 * @brief checks if the given ContactLane is valid
 *
 * The contactLane is valid if it's within valid input range.
 */
inline bool isValid(ContactLane const &contactLane,
                    bool const logErrors = true) {
  return withinValidInputRange(contactLane, logErrors);
}

}  // namespace lane
}  // namespace map
}  // namespace ad
