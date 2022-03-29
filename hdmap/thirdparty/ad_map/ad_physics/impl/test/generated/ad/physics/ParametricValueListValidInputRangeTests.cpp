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

#include "ad/physics/ParametricValueListValidInputRange.hpp"

TEST(ParametricValueListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::ParametricValueList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricValueListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::ParametricValueList value;
  ::ad::physics::ParametricValue element(0.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricValueListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::ParametricValueList value;
  ::ad::physics::ParametricValue element(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
