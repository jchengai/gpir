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

#include "ad/physics/RatioValueListValidInputRange.hpp"

TEST(RatioValueListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::RatioValueList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RatioValueListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::RatioValueList value;
  ::ad::physics::RatioValue element(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
