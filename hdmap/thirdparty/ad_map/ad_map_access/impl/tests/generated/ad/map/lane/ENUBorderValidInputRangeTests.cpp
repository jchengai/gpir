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

#include "ad/map/lane/ENUBorderValidInputRange.hpp"

TEST(ENUBorderValidInputRangeTests, testValidInputRange)
{
  ::ad::map::lane::ENUBorder value;
  ::ad::map::point::ENUEdge valueLeft;
  ::ad::map::point::ENUPoint valueLeftElement;
  ::ad::map::point::ENUCoordinate valueLeftElementX(-16384);
  valueLeftElement.x = valueLeftElementX;
  ::ad::map::point::ENUCoordinate valueLeftElementY(-16384);
  valueLeftElement.y = valueLeftElementY;
  ::ad::map::point::ENUCoordinate valueLeftElementZ(-16384);
  valueLeftElement.z = valueLeftElementZ;
  valueLeft.resize(1, valueLeftElement);
  value.left = valueLeft;
  ::ad::map::point::ENUEdge valueRight;
  ::ad::map::point::ENUPoint valueRightElement;
  ::ad::map::point::ENUCoordinate valueRightElementX(-16384);
  valueRightElement.x = valueRightElementX;
  ::ad::map::point::ENUCoordinate valueRightElementY(-16384);
  valueRightElement.y = valueRightElementY;
  ::ad::map::point::ENUCoordinate valueRightElementZ(-16384);
  valueRightElement.z = valueRightElementZ;
  valueRight.resize(1, valueRightElement);
  value.right = valueRight;
  ASSERT_TRUE(withinValidInputRange(value));
}
