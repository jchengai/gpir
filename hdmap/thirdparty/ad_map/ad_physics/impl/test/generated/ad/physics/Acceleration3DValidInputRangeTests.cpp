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

#include "ad/physics/Acceleration3DValidInputRange.hpp"

TEST(Acceleration3DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Acceleration invalidInitializedMember(-1e2 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Acceleration invalidInitializedMember(1e2 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Acceleration invalidInitializedMember(-1e2 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Acceleration invalidInitializedMember(1e2 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Acceleration invalidInitializedMember(-1e2 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueX(-1e2);
  value.x = valueX;
  ::ad::physics::Acceleration valueY(-1e2);
  value.y = valueY;
  ::ad::physics::Acceleration valueZ(-1e2);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Acceleration invalidInitializedMember(1e2 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Acceleration3DValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::physics::Acceleration3D value;
  ::ad::physics::Acceleration valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
