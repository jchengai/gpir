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

#include "ad/physics/SpeedListValidInputRange.hpp"

TEST(SpeedListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::SpeedList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::SpeedList value;
  ::ad::physics::Speed element(-100.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::SpeedList value;
  ::ad::physics::Speed element(-100. * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
