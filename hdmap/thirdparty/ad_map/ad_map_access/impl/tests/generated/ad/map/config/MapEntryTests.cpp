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
#include "ad/map/config/MapEntry.hpp"

class MapEntryTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::config::MapEntry value;
    std::string valueFilename{"min"};
    value.filename = valueFilename;
    ::ad::physics::Distance valueOpenDriveOverlapMargin(-1e9);
    value.openDriveOverlapMargin = valueOpenDriveOverlapMargin;
    ::ad::map::intersection::IntersectionType valueOpenDriveDefaultIntersectionType(
      ::ad::map::intersection::IntersectionType::Unknown);
    value.openDriveDefaultIntersectionType = valueOpenDriveDefaultIntersectionType;
    ::ad::map::landmark::TrafficLightType valueOpenDriveDefaultTrafficLightType(
      ::ad::map::landmark::TrafficLightType::INVALID);
    value.openDriveDefaultTrafficLightType = valueOpenDriveDefaultTrafficLightType;
    mValue = value;
  }

  ::ad::map::config::MapEntry mValue;
};

TEST_F(MapEntryTests, copyConstruction)
{
  ::ad::map::config::MapEntry value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapEntryTests, moveConstruction)
{
  ::ad::map::config::MapEntry tmpValue(mValue);
  ::ad::map::config::MapEntry value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(MapEntryTests, copyAssignment)
{
  ::ad::map::config::MapEntry value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(MapEntryTests, moveAssignment)
{
  ::ad::map::config::MapEntry tmpValue(mValue);
  ::ad::map::config::MapEntry value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapEntryTests, comparisonOperatorEqual)
{
  ::ad::map::config::MapEntry valueA = mValue;
  ::ad::map::config::MapEntry valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(MapEntryTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(MapEntryTests, comparisonOperatorFilenameDiffers)
{
  ::ad::map::config::MapEntry valueA = mValue;
  std::string filename{"max"};
  valueA.filename = filename;
  ::ad::map::config::MapEntry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapEntryTests, comparisonOperatorOpenDriveOverlapMarginDiffers)
{
  ::ad::map::config::MapEntry valueA = mValue;
  ::ad::physics::Distance openDriveOverlapMargin(1e9);
  valueA.openDriveOverlapMargin = openDriveOverlapMargin;
  ::ad::map::config::MapEntry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapEntryTests, comparisonOperatorOpenDriveDefaultIntersectionTypeDiffers)
{
  ::ad::map::config::MapEntry valueA = mValue;
  ::ad::map::intersection::IntersectionType openDriveDefaultIntersectionType(
    ::ad::map::intersection::IntersectionType::TrafficLight);
  valueA.openDriveDefaultIntersectionType = openDriveDefaultIntersectionType;
  ::ad::map::config::MapEntry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapEntryTests, comparisonOperatorOpenDriveDefaultTrafficLightTypeDiffers)
{
  ::ad::map::config::MapEntry valueA = mValue;
  ::ad::map::landmark::TrafficLightType openDriveDefaultTrafficLightType(
    ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);
  valueA.openDriveDefaultTrafficLightType = openDriveDefaultTrafficLightType;
  ::ad::map::config::MapEntry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
