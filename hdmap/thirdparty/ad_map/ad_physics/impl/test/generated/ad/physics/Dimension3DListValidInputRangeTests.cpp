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

#include "ad/physics/Dimension3DListValidInputRange.hpp"

TEST(Dimension3DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::Dimension3DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension3DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::Dimension3DList value;
  ::ad::physics::Dimension3D element;
  ::ad::physics::Distance elementLength(-1e9);
  element.length = elementLength;
  ::ad::physics::Distance elementWidth(-1e9);
  element.width = elementWidth;
  ::ad::physics::Distance elementHeight(-1e9);
  element.height = elementHeight;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension3DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::Dimension3DList value;
  ::ad::physics::Dimension3D element;
  ::ad::physics::Distance elementLength(-1e9 * 1.1);
  element.length = elementLength;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
