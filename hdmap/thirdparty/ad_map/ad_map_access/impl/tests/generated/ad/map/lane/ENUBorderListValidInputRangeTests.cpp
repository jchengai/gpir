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

#include "ad/map/lane/ENUBorderListValidInputRange.hpp"

TEST(ENUBorderListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::ENUBorderList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUBorderListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::ENUBorderList value;
  ::ad::map::lane::ENUBorder element;
  ::ad::map::point::ENUEdge elementLeft;
  ::ad::map::point::ENUPoint elementLeftElement;
  ::ad::map::point::ENUCoordinate elementLeftElementX(-16384);
  elementLeftElement.x = elementLeftElementX;
  ::ad::map::point::ENUCoordinate elementLeftElementY(-16384);
  elementLeftElement.y = elementLeftElementY;
  ::ad::map::point::ENUCoordinate elementLeftElementZ(-16384);
  elementLeftElement.z = elementLeftElementZ;
  elementLeft.resize(1, elementLeftElement);
  element.left = elementLeft;
  ::ad::map::point::ENUEdge elementRight;
  ::ad::map::point::ENUPoint elementRightElement;
  ::ad::map::point::ENUCoordinate elementRightElementX(-16384);
  elementRightElement.x = elementRightElementX;
  ::ad::map::point::ENUCoordinate elementRightElementY(-16384);
  elementRightElement.y = elementRightElementY;
  ::ad::map::point::ENUCoordinate elementRightElementZ(-16384);
  elementRightElement.z = elementRightElementZ;
  elementRight.resize(1, elementRightElement);
  element.right = elementRight;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
