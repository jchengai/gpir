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

#include "ad/map/point/ECEFEdgeValidInputRange.hpp"

TEST(ECEFEdgeValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::point::ECEFEdge value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFEdgeValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::point::ECEFEdge value;
  ::ad::map::point::ECEFPoint element;
  ::ad::map::point::ECEFCoordinate elementX(-6400000);
  element.x = elementX;
  ::ad::map::point::ECEFCoordinate elementY(-6400000);
  element.y = elementY;
  ::ad::map::point::ECEFCoordinate elementZ(-6400000);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFEdgeValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::point::ECEFEdge value;
  ::ad::map::point::ECEFPoint element;
  ::ad::map::point::ECEFCoordinate elementX(-6400000 * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
