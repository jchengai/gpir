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

#include "ad/physics/ProbabilityListValidInputRange.hpp"

TEST(ProbabilityListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::ProbabilityList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ProbabilityListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::ProbabilityList value;
  ::ad::physics::Probability element(0.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ProbabilityListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::ProbabilityList value;
  ::ad::physics::Probability element(0. - ::ad::physics::Probability::cPrecisionValue);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
