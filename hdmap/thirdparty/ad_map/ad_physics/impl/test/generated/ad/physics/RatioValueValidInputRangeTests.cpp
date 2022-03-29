/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/*
 * Generated file
 */

#include <gtest/gtest.h>

#include <limits>

#include "ad/physics/RatioValueValidInputRange.hpp"

TEST(RatioValueValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::RatioValue value;
  ASSERT_FALSE(withinValidInputRange(value));
}
