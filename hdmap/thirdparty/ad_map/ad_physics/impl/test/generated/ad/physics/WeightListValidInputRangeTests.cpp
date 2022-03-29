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

#include "ad/physics/WeightListValidInputRange.hpp"

TEST(WeightListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::WeightList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(WeightListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::WeightList value;
  ::ad::physics::Weight element(std::numeric_limits<::ad::physics::Weight>::lowest());
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(WeightListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::WeightList value;
  ::ad::physics::Weight element{}; // TODO: not invalid
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
