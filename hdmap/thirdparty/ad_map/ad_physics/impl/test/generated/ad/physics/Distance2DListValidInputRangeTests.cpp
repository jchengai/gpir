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

#include "ad/physics/Distance2DListValidInputRange.hpp"

TEST(Distance2DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::Distance2DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance2DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::Distance2DList value;
  ::ad::physics::Distance2D element;
  ::ad::physics::Distance elementX(-1e9);
  element.x = elementX;
  ::ad::physics::Distance elementY(-1e9);
  element.y = elementY;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance2DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::Distance2DList value;
  ::ad::physics::Distance2D element;
  ::ad::physics::Distance elementX(-1e9 * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
