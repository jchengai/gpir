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

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

#include <gtest/gtest.h>
#include <limits>
#include "ad/map/landmark/ENULandmark.hpp"

class ENULandmarkTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
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
    mValue = value;
  }

  ::ad::map::landmark::ENULandmark mValue;
};

TEST_F(ENULandmarkTests, copyConstruction)
{
  ::ad::map::landmark::ENULandmark value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENULandmarkTests, moveConstruction)
{
  ::ad::map::landmark::ENULandmark tmpValue(mValue);
  ::ad::map::landmark::ENULandmark value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ENULandmarkTests, copyAssignment)
{
  ::ad::map::landmark::ENULandmark value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ENULandmarkTests, moveAssignment)
{
  ::ad::map::landmark::ENULandmark tmpValue(mValue);
  ::ad::map::landmark::ENULandmark value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENULandmarkTests, comparisonOperatorEqual)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ENULandmarkTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ENULandmarkTests, comparisonOperatorIdDiffers)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::landmark::LandmarkId id(std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  valueA.id = id;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENULandmarkTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::landmark::LandmarkType type(::ad::map::landmark::LandmarkType::OTHER);
  valueA.type = type;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENULandmarkTests, comparisonOperatorPositionDiffers)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::point::ENUPoint position;
  ::ad::map::point::ENUCoordinate positionX(16384);
  position.x = positionX;
  ::ad::map::point::ENUCoordinate positionY(16384);
  position.y = positionY;
  ::ad::map::point::ENUCoordinate positionZ(16384);
  position.z = positionZ;
  valueA.position = position;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENULandmarkTests, comparisonOperatorHeadingDiffers)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::point::ENUHeading heading(3.141592655);
  valueA.heading = heading;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENULandmarkTests, comparisonOperatorTrafficLightTypeDiffers)
{
  ::ad::map::landmark::ENULandmark valueA = mValue;
  ::ad::map::landmark::TrafficLightType trafficLightType(
    ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);
  valueA.trafficLightType = trafficLightType;
  ::ad::map::landmark::ENULandmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
