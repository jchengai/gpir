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

#include "ad/physics/AngularAccelerationListValidInputRange.hpp"

TEST(AngularAccelerationListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AngularAccelerationList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularAccelerationListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AngularAccelerationList value;
  ::ad::physics::AngularAcceleration element(-1e2);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularAccelerationListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AngularAccelerationList value;
  ::ad::physics::AngularAcceleration element(-1e2 * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
