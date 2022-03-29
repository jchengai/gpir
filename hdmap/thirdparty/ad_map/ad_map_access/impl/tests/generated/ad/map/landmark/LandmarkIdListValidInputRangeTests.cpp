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

#include "ad/map/landmark/LandmarkIdListValidInputRange.hpp"

TEST(LandmarkIdListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::landmark::LandmarkIdList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LandmarkIdListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::landmark::LandmarkIdList value;
  ::ad::map::landmark::LandmarkId element(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
