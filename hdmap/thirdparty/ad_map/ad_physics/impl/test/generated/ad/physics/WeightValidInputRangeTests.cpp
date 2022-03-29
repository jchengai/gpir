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

#include "ad/physics/WeightValidInputRange.hpp"

TEST(WeightValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Weight value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(WeightValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Weight value(48600.0);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(WeightValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Weight value(48600.0 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
