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

#include "ad/map/restriction/RoadUserTypeListValidInputRange.hpp"

TEST(RoadUserTypeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::restriction::RoadUserTypeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RoadUserTypeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::restriction::RoadUserTypeList value;
  ::ad::map::restriction::RoadUserType element(::ad::map::restriction::RoadUserType::INVALID);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RoadUserTypeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::restriction::RoadUserTypeList value;
  ::ad::map::restriction::RoadUserType element(static_cast<::ad::map::restriction::RoadUserType>(-1));
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
