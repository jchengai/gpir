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

#include "ad/map/lane/ECEFBorderListValidInputRange.hpp"

TEST(ECEFBorderListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::ECEFBorderList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFBorderListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::ECEFBorderList value;
  ::ad::map::lane::ECEFBorder element;
  ::ad::map::point::ECEFEdge elementLeft;
  ::ad::map::point::ECEFPoint elementLeftElement;
  ::ad::map::point::ECEFCoordinate elementLeftElementX(-6400000);
  elementLeftElement.x = elementLeftElementX;
  ::ad::map::point::ECEFCoordinate elementLeftElementY(-6400000);
  elementLeftElement.y = elementLeftElementY;
  ::ad::map::point::ECEFCoordinate elementLeftElementZ(-6400000);
  elementLeftElement.z = elementLeftElementZ;
  elementLeft.resize(1, elementLeftElement);
  element.left = elementLeft;
  ::ad::map::point::ECEFEdge elementRight;
  ::ad::map::point::ECEFPoint elementRightElement;
  ::ad::map::point::ECEFCoordinate elementRightElementX(-6400000);
  elementRightElement.x = elementRightElementX;
  ::ad::map::point::ECEFCoordinate elementRightElementY(-6400000);
  elementRightElement.y = elementRightElementY;
  ::ad::map::point::ECEFCoordinate elementRightElementZ(-6400000);
  elementRightElement.z = elementRightElementZ;
  elementRight.resize(1, elementRightElement);
  element.right = elementRight;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
