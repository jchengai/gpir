// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/restriction/RestrictionValidInputRange.hpp"
#include "ad/map/restriction/RestrictionsValidInputRange.hpp"
#include "ad/map/restriction/VehicleDescriptorValidInputRange.hpp"

namespace ad {
namespace map {
namespace restriction {

/**
 * @brief checks if the given VehicleDescriptor is valid
 *
 * The descriptor is valid if it's within valid input range.
 */
inline bool isValid(VehicleDescriptor const &descriptor,
                    bool const logErrors = true) {
  return withinValidInputRange(descriptor, logErrors);
}

/**
 * @brief checks if the given Restriction is valid
 *
 * The restriction is valid if it's within valid input range.
 */
inline bool isValid(Restriction const &restriction,
                    bool const logErrors = true) {
  return withinValidInputRange(restriction, logErrors);
}

/**
 * @brief checks if the given Restrictions is valid
 *
 * The restrictions is valid if it's within valid input range.
 */
inline bool isValid(Restrictions const &restrictions,
                    bool const logErrors = true) {
  return withinValidInputRange(restrictions, logErrors);
}

/**
 * @brief Checks if restriction allows vehicle at this object.
 * @param[in] restriction data.
 * @param[in] vehicle Vehicle data.
 */
bool isAccessOk(Restriction const &restriction,
                VehicleDescriptor const &vehicle);

/**
 * @brief Checks if vehicle fits the restriction criteria.
 * @param vehicle Description of the vehicle.
 * @returns true of vehicle fits the restrictions criteria.
 */
bool isAccessOk(Restrictions const &restrictions,
                VehicleDescriptor const &vehicle);

/** @returns Maximum of HOV restriction found. */
PassengerCount getHOV(Restrictions const &restrictions);

}  // namespace restriction
}  // namespace map
}  // namespace ad
