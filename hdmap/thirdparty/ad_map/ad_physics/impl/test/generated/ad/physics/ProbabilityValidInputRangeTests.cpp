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

#include "ad/physics/ProbabilityValidInputRange.hpp"

TEST(ProbabilityValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Probability value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ProbabilityValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Probability value(0.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ProbabilityValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Probability value(1.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ProbabilityValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Probability value(0. - ::ad::physics::Probability::cPrecisionValue);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ProbabilityValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Probability value(1. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
