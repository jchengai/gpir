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

#include "ad/physics/SpeedRangeListValidInputRange.hpp"

TEST(SpeedRangeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::SpeedRangeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedRangeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::SpeedRangeList value;
  ::ad::physics::SpeedRange element;
  ::ad::physics::Speed elementMinimum(-100.);
  element.minimum = elementMinimum;
  ::ad::physics::Speed elementMaximum(-100.);
  element.maximum = elementMaximum;
  element.maximum = element.minimum;
  element.minimum = element.maximum;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedRangeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::SpeedRangeList value;
  ::ad::physics::SpeedRange element;
  ::ad::physics::Speed elementMinimum(-100. * 1.1);
  element.minimum = elementMinimum;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
