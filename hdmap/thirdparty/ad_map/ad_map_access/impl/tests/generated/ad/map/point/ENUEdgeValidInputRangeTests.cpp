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

#include "ad/map/point/ENUEdgeValidInputRange.hpp"

TEST(ENUEdgeValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::point::ENUEdge value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUEdgeValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::point::ENUEdge value;
  ::ad::map::point::ENUPoint element;
  ::ad::map::point::ENUCoordinate elementX(-16384);
  element.x = elementX;
  ::ad::map::point::ENUCoordinate elementY(-16384);
  element.y = elementY;
  ::ad::map::point::ENUCoordinate elementZ(-16384);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUEdgeValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::point::ENUEdge value;
  ::ad::map::point::ENUPoint element;
  ::ad::map::point::ENUCoordinate elementX(-16384 * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
