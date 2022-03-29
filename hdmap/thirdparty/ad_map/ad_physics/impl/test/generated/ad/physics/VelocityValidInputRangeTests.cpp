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

#include "ad/physics/VelocityValidInputRange.hpp"

TEST(VelocityValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueX(-100.);
  value.x = valueX;
  ::ad::physics::Speed valueY(-100.);
  value.y = valueY;
  ::ad::physics::Speed valueZ(-100.);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VelocityValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::physics::Velocity value;
  ::ad::physics::Speed valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
