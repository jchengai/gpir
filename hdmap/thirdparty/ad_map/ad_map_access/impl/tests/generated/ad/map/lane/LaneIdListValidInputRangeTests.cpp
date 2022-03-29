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

#include "ad/map/lane/LaneIdListValidInputRange.hpp"

TEST(LaneIdListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::LaneIdList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneIdListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::LaneIdList value;
  ::ad::map::lane::LaneId element(1);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
