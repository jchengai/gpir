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

#include "ad/physics/AccelerationListValidInputRange.hpp"

TEST(AccelerationListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AccelerationList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AccelerationList value;
  ::ad::physics::Acceleration element(-1e2);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AccelerationList value;
  ::ad::physics::Acceleration element(-1e2 * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
