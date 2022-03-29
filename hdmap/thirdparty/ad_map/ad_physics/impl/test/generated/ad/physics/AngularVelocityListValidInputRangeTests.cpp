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

#include "ad/physics/AngularVelocityListValidInputRange.hpp"

TEST(AngularVelocityListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AngularVelocityList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocityListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AngularVelocityList value;
  ::ad::physics::AngularVelocity element(-100.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocityListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AngularVelocityList value;
  ::ad::physics::AngularVelocity element(-100. * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
