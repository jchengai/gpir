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

#include "ad/map/point/BoundingSphereValidInputRange.hpp"

TEST(BoundingSphereValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::map::point::ECEFPoint valueCenter;
  ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
  valueCenter.x = valueCenterX;
  ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
  valueCenter.y = valueCenterY;
  ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
  valueCenter.z = valueCenterZ;
  value.center = valueCenter;
  ::ad::physics::Distance valueRadius(-1e9);
  value.radius = valueRadius;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(BoundingSphereValidInputRangeTests, testValidInputRangeCenterTooSmall)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::map::point::ECEFPoint valueCenter;
  ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
  valueCenter.x = valueCenterX;
  ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
  valueCenter.y = valueCenterY;
  ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
  valueCenter.z = valueCenterZ;
  value.center = valueCenter;
  ::ad::physics::Distance valueRadius(-1e9);
  value.radius = valueRadius;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(-6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.center = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(BoundingSphereValidInputRangeTests, testValidInputRangeCenterTooBig)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::map::point::ECEFPoint valueCenter;
  ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
  valueCenter.x = valueCenterX;
  ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
  valueCenter.y = valueCenterY;
  ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
  valueCenter.z = valueCenterZ;
  value.center = valueCenter;
  ::ad::physics::Distance valueRadius(-1e9);
  value.radius = valueRadius;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.center = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(BoundingSphereValidInputRangeTests, testValidInputRangeRadiusTooSmall)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::map::point::ECEFPoint valueCenter;
  ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
  valueCenter.x = valueCenterX;
  ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
  valueCenter.y = valueCenterY;
  ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
  valueCenter.z = valueCenterZ;
  value.center = valueCenter;
  ::ad::physics::Distance valueRadius(-1e9);
  value.radius = valueRadius;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.radius = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(BoundingSphereValidInputRangeTests, testValidInputRangeRadiusTooBig)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::map::point::ECEFPoint valueCenter;
  ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
  valueCenter.x = valueCenterX;
  ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
  valueCenter.y = valueCenterY;
  ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
  valueCenter.z = valueCenterZ;
  value.center = valueCenter;
  ::ad::physics::Distance valueRadius(-1e9);
  value.radius = valueRadius;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.radius = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(BoundingSphereValidInputRangeTests, testValidInputRangeradiusDefault)
{
  ::ad::map::point::BoundingSphere value;
  ::ad::physics::Distance valueDefault;
  value.radius = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
