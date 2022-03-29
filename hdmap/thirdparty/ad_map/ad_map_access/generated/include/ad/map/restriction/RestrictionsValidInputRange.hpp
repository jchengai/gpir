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

#include <cmath>
#include <limits>
#include "ad/map/restriction/RestrictionListValidInputRange.hpp"
#include "ad/map/restriction/Restrictions.hpp"

/*!
 * \brief check if the given Restrictions is within valid input range
 *
 * \param[in] input the Restrictions as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Restrictions is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::restriction::Restrictions const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  (void)input;
  (void)logErrors;
  bool inValidInputRange = true;
  return inValidInputRange;
}
