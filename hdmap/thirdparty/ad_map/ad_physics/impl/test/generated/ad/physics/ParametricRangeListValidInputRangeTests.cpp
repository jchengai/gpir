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

#include "ad/physics/ParametricRangeListValidInputRange.hpp"

TEST(ParametricRangeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::ParametricRangeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricRangeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::ParametricRangeList value;
  ::ad::physics::ParametricRange element;
  ::ad::physics::ParametricValue elementMinimum(0.);
  element.minimum = elementMinimum;
  ::ad::physics::ParametricValue elementMaximum(0.);
  element.maximum = elementMaximum;
  element.maximum = element.minimum;
  element.minimum = element.maximum;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricRangeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::ParametricRangeList value;
  ::ad::physics::ParametricRange element;
  ::ad::physics::ParametricValue elementMinimum(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  element.minimum = elementMinimum;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
