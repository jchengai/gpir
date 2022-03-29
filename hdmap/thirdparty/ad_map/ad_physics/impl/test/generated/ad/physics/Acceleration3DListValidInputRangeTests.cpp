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

#include "ad/physics/Acceleration3DListValidInputRange.hpp"

TEST(Acceleration3DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::Acceleration3DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Acceleration3DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::Acceleration3DList value;
  ::ad::physics::Acceleration3D element;
  ::ad::physics::Acceleration elementX(-1e2);
  element.x = elementX;
  ::ad::physics::Acceleration elementY(-1e2);
  element.y = elementY;
  ::ad::physics::Acceleration elementZ(-1e2);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Acceleration3DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::Acceleration3DList value;
  ::ad::physics::Acceleration3D element;
  ::ad::physics::Acceleration elementX(-1e2 * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
