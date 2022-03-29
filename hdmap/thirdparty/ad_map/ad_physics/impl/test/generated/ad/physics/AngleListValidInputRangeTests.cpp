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

#include "ad/physics/AngleListValidInputRange.hpp"

TEST(AngleListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AngleList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AngleList value;
  ::ad::physics::Angle element(-6.283185308);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AngleList value;
  ::ad::physics::Angle element(-6.283185308 * 1.1);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
