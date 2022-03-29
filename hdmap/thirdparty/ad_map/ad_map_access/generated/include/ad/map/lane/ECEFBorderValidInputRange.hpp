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
#include "ad/map/lane/ECEFBorder.hpp"
#include "ad/map/point/ECEFEdgeValidInputRange.hpp"

/*!
 * \brief check if the given ECEFBorder is within valid input range
 *
 * \param[in] input the ECEFBorder as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ECEFBorder is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::lane::ECEFBorder const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  (void)input;
  (void)logErrors;
  bool inValidInputRange = true;
  return inValidInputRange;
}
