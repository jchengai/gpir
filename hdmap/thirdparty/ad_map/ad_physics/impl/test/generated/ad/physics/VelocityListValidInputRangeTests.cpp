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

#include "ad/physics/VelocityListValidInputRange.hpp"

TEST(VelocityListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::VelocityList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(VelocityListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::VelocityList value;
  ::ad::physics::Velocity element;
  ::ad::physics::Speed elementX(-100.);
  element.x = elementX;
  ::ad::physics::Speed elementY(-100.);
  element.y = elementY;
  ::ad::physics::Speed elementZ(-100.);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(VelocityListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::VelocityList value;
  ::ad::physics::Velocity element;
  ::ad::physics::Speed elementX(-100. * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
