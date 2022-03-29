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

#include "ad/physics/DurationListValidInputRange.hpp"

TEST(DurationListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::DurationList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::DurationList value;
  ::ad::physics::Duration element(0.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::DurationList value;
  ::ad::physics::Duration element(0. - ::ad::physics::Duration::cPrecisionValue);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
