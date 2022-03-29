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

#include "ad/physics/AngularVelocity3DValidInputRange.hpp"

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::AngularVelocity invalidInitializedMember(-100. * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::AngularVelocity invalidInitializedMember(100. * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::AngularVelocity invalidInitializedMember(-100. * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::AngularVelocity invalidInitializedMember(100. * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::AngularVelocity invalidInitializedMember(-100. * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueX(-100.);
  value.x = valueX;
  ::ad::physics::AngularVelocity valueY(-100.);
  value.y = valueY;
  ::ad::physics::AngularVelocity valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::AngularVelocity invalidInitializedMember(100. * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocity3DValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::physics::AngularVelocity3D value;
  ::ad::physics::AngularVelocity valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
