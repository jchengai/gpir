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

#include "ad/map/landmark/ENULandmarkValidInputRange.hpp"

TEST(ENULandmarkValidInputRangeTests, testValidInputRange)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value below input range minimum
  ::ad::map::landmark::LandmarkType invalidInitializedMember(static_cast<::ad::map::landmark::LandmarkType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeTypeTooBig)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value above input range maximum
  ::ad::map::landmark::LandmarkType invalidInitializedMember(static_cast<::ad::map::landmark::LandmarkType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangePositionTooSmall)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUPoint invalidInitializedMember;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberX(-16384 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.position = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangePositionTooBig)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUPoint invalidInitializedMember;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberX(16384 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.position = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeHeadingTooSmall)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUHeading invalidInitializedMember(-3.141592655 * 1.1);
  value.heading = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeHeadingTooBig)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUHeading invalidInitializedMember(3.141592655 * 1.1);
  value.heading = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeheadingDefault)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::point::ENUHeading valueDefault;
  value.heading = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeTrafficLightTypeTooSmall)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value below input range minimum
  ::ad::map::landmark::TrafficLightType invalidInitializedMember(
    static_cast<::ad::map::landmark::TrafficLightType>(-1));
  value.trafficLightType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENULandmarkValidInputRangeTests, testValidInputRangeTrafficLightTypeTooBig)
{
  ::ad::map::landmark::ENULandmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ENUPoint valuePosition;
  ::ad::map::point::ENUCoordinate valuePositionX(-16384);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ENUCoordinate valuePositionY(-16384);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ENUCoordinate valuePositionZ(-16384);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;

  // override member with data type value above input range maximum
  ::ad::map::landmark::TrafficLightType invalidInitializedMember(
    static_cast<::ad::map::landmark::TrafficLightType>(-1));
  value.trafficLightType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
