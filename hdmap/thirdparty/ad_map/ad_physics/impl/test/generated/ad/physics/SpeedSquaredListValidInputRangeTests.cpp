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

#include "ad/physics/SpeedSquaredListValidInputRange.hpp"

TEST(SpeedSquaredListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::SpeedSquaredList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedSquaredListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::SpeedSquaredList value;
  ::ad::physics::SpeedSquared element(-1e4);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedSquaredListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::SpeedSquaredList value;
  ::ad::physics::SpeedSquared element(-1e4 * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
