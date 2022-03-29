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

#include "ad/physics/MetricRangeListValidInputRange.hpp"

TEST(MetricRangeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::MetricRangeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MetricRangeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::MetricRangeList value;
  ::ad::physics::MetricRange element;
  ::ad::physics::Distance elementMinimum(-1e9);
  elementMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  element.minimum = elementMinimum;
  ::ad::physics::Distance elementMaximum(-1e9);
  element.maximum = elementMaximum;
  element.maximum = element.minimum;
  element.minimum = element.maximum;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MetricRangeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::MetricRangeList value;
  ::ad::physics::MetricRange element;
  ::ad::physics::Distance elementMinimum(-1e9 * 1.1);
  element.minimum = elementMinimum;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
