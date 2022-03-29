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

#include "ad/map/lane/ECEFBorderValidInputRange.hpp"

TEST(ECEFBorderValidInputRangeTests, testValidInputRange)
{
  ::ad::map::lane::ECEFBorder value;
  ::ad::map::point::ECEFEdge valueLeft;
  ::ad::map::point::ECEFPoint valueLeftElement;
  ::ad::map::point::ECEFCoordinate valueLeftElementX(-6400000);
  valueLeftElement.x = valueLeftElementX;
  ::ad::map::point::ECEFCoordinate valueLeftElementY(-6400000);
  valueLeftElement.y = valueLeftElementY;
  ::ad::map::point::ECEFCoordinate valueLeftElementZ(-6400000);
  valueLeftElement.z = valueLeftElementZ;
  valueLeft.resize(1, valueLeftElement);
  value.left = valueLeft;
  ::ad::map::point::ECEFEdge valueRight;
  ::ad::map::point::ECEFPoint valueRightElement;
  ::ad::map::point::ECEFCoordinate valueRightElementX(-6400000);
  valueRightElement.x = valueRightElementX;
  ::ad::map::point::ECEFCoordinate valueRightElementY(-6400000);
  valueRightElement.y = valueRightElementY;
  ::ad::map::point::ECEFCoordinate valueRightElementZ(-6400000);
  valueRightElement.z = valueRightElementZ;
  valueRight.resize(1, valueRightElement);
  value.right = valueRight;
  ASSERT_TRUE(withinValidInputRange(value));
}
