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

#include "ad/physics/DistanceSquaredListValidInputRange.hpp"

TEST(DistanceSquaredListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::DistanceSquaredList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceSquaredListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::DistanceSquaredList value;
  ::ad::physics::DistanceSquared element(0.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceSquaredListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::DistanceSquaredList value;
  ::ad::physics::DistanceSquared element(0. - ::ad::physics::DistanceSquared::cPrecisionValue);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
