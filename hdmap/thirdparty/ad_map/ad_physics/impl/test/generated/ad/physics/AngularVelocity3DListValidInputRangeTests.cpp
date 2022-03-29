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

#include "ad/physics/AngularVelocity3DListValidInputRange.hpp"

TEST(AngularVelocity3DListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::physics::AngularVelocity3DList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocity3DListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::physics::AngularVelocity3DList value;
  ::ad::physics::AngularVelocity3D element;
  ::ad::physics::AngularVelocity elementX(-100.);
  element.x = elementX;
  ::ad::physics::AngularVelocity elementY(-100.);
  element.y = elementY;
  ::ad::physics::AngularVelocity elementZ(-100.);
  element.z = elementZ;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocity3DListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::physics::AngularVelocity3DList value;
  ::ad::physics::AngularVelocity3D element;
  ::ad::physics::AngularVelocity elementX(-100. * 1.1);
  element.x = elementX;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
