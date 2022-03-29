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

#include "ad/physics/ParametricValueValidInputRange.hpp"

TEST(ParametricValueValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::ParametricValue value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricValueValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::ParametricValue value(0.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricValueValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::ParametricValue value(1.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricValueValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::ParametricValue value(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricValueValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::ParametricValue value(1. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
