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

#include "ad/map/point/ParaPointListValidInputRange.hpp"

TEST(ParaPointListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::point::ParaPointList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParaPointListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::point::ParaPointList value;
  ::ad::map::point::ParaPoint element;
  ::ad::map::lane::LaneId elementLaneId(1);
  element.laneId = elementLaneId;
  ::ad::physics::ParametricValue elementParametricOffset(0.);
  element.parametricOffset = elementParametricOffset;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParaPointListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::point::ParaPointList value;
  ::ad::map::point::ParaPoint element;
  ::ad::physics::ParametricValue elementParametricOffset(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  element.parametricOffset = elementParametricOffset;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
