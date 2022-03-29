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

#include "ad/physics/DistanceListValidInputRange.hpp"

TEST(DistanceListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::DistanceList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::DistanceList value;
  ::ad::physics::Distance element(-1e9);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::DistanceList value;
  ::ad::physics::Distance element(-1e9 * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
