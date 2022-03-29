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

#include "ad/physics/AngleRangeListValidInputRange.hpp"

TEST(AngleRangeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AngleRangeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleRangeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AngleRangeList value;
  ::ad::physics::AngleRange element;
  ::ad::physics::Angle elementMinimum(-6.283185308);
  element.minimum = elementMinimum;
  ::ad::physics::Angle elementMaximum(-6.283185308);
  element.maximum = elementMaximum;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleRangeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AngleRangeList value;
  ::ad::physics::AngleRange element;
  ::ad::physics::Angle elementMinimum(-6.283185308 * 1.1);
  element.minimum = elementMinimum;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
