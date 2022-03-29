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

#include "ad/physics/Distance3DValidInputRange.hpp"

TEST(Distance3DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ::ad::physics::Distance valueZ(-1e9);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance3DValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::physics::Distance3D value;
  ::ad::physics::Distance valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
