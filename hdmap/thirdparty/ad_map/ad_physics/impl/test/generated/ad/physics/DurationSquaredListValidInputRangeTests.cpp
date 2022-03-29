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

#include "ad/physics/DurationSquaredListValidInputRange.hpp"

TEST(DurationSquaredListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::DurationSquaredList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationSquaredListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::DurationSquaredList value;
  ::ad::physics::DurationSquared element(0.);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationSquaredListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::DurationSquaredList value;
  ::ad::physics::DurationSquared element(0. - ::ad::physics::DurationSquared::cPrecisionValue);
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
