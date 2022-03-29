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

#include "ad/physics/Distance3DListValidInputRange.hpp"

TEST(Distance3DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::Distance3DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance3DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::Distance3DList value;
  ::ad::physics::Distance3D element;
  ::ad::physics::Distance elementX(-1e9);
  element.x = elementX;
  ::ad::physics::Distance elementY(-1e9);
  element.y = elementY;
  ::ad::physics::Distance elementZ(-1e9);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance3DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::Distance3DList value;
  ::ad::physics::Distance3D element;
  ::ad::physics::Distance elementX(-1e9 * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
