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

#include "ad/physics/Dimension2DListValidInputRange.hpp"

TEST(Dimension2DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::Dimension2DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension2DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::Dimension2DList value;
  ::ad::physics::Dimension2D element;
  ::ad::physics::Distance elementLength(-1e9);
  element.length = elementLength;
  ::ad::physics::Distance elementWidth(-1e9);
  element.width = elementWidth;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension2DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::Dimension2DList value;
  ::ad::physics::Dimension2D element;
  ::ad::physics::Distance elementLength(-1e9 * 1.1);
  element.length = elementLength;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
