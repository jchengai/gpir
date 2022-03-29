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

#include "ad/physics/AccelerationRangeListValidInputRange.hpp"

TEST(AccelerationRangeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AccelerationRangeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationRangeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AccelerationRangeList value;
  ::ad::physics::AccelerationRange element;
  ::ad::physics::Acceleration elementMinimum(-1e2);
  element.minimum = elementMinimum;
  ::ad::physics::Acceleration elementMaximum(-1e2);
  element.maximum = elementMaximum;
  element.maximum = element.minimum;
  element.minimum = element.maximum;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationRangeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AccelerationRangeList value;
  ::ad::physics::AccelerationRange element;
  ::ad::physics::Acceleration elementMinimum(-1e2 * 1.1);
  element.minimum = elementMinimum;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
