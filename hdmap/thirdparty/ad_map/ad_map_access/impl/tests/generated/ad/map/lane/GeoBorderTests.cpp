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
#include "ad/map/lane/GeoBorder.hpp"

class GeoBorderTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::lane::GeoBorder value;
    ::ad::map::point::GeoEdge valueLeft;
    ::ad::map::point::GeoPoint valueLeftElement;
    ::ad::map::point::Longitude valueLeftElementLongitude(-180);
    valueLeftElement.longitude = valueLeftElementLongitude;
    ::ad::map::point::Latitude valueLeftElementLatitude(-90);
    valueLeftElement.latitude = valueLeftElementLatitude;
    ::ad::map::point::Altitude valueLeftElementAltitude(-11000);
    valueLeftElement.altitude = valueLeftElementAltitude;
    valueLeft.resize(1, valueLeftElement);
    value.left = valueLeft;
    ::ad::map::point::GeoEdge valueRight;
    ::ad::map::point::GeoPoint valueRightElement;
    ::ad::map::point::Longitude valueRightElementLongitude(-180);
    valueRightElement.longitude = valueRightElementLongitude;
    ::ad::map::point::Latitude valueRightElementLatitude(-90);
    valueRightElement.latitude = valueRightElementLatitude;
    ::ad::map::point::Altitude valueRightElementAltitude(-11000);
    valueRightElement.altitude = valueRightElementAltitude;
    valueRight.resize(1, valueRightElement);
    value.right = valueRight;
    mValue = value;
  }

  ::ad::map::lane::GeoBorder mValue;
};

TEST_F(GeoBorderTests, copyConstruction)
{
  ::ad::map::lane::GeoBorder value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoBorderTests, moveConstruction)
{
  ::ad::map::lane::GeoBorder tmpValue(mValue);
  ::ad::map::lane::GeoBorder value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoBorderTests, copyAssignment)
{
  ::ad::map::lane::GeoBorder value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoBorderTests, moveAssignment)
{
  ::ad::map::lane::GeoBorder tmpValue(mValue);
  ::ad::map::lane::GeoBorder value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoBorderTests, comparisonOperatorEqual)
{
  ::ad::map::lane::GeoBorder valueA = mValue;
  ::ad::map::lane::GeoBorder valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(GeoBorderTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(GeoBorderTests, comparisonOperatorLeftDiffers)
{
  ::ad::map::lane::GeoBorder valueA = mValue;
  ::ad::map::point::GeoEdge left;
  ::ad::map::point::GeoPoint leftElement;
  ::ad::map::point::Longitude leftElementLongitude(180);
  leftElement.longitude = leftElementLongitude;
  ::ad::map::point::Latitude leftElementLatitude(90);
  leftElement.latitude = leftElementLatitude;
  ::ad::map::point::Altitude leftElementAltitude(9000);
  leftElement.altitude = leftElementAltitude;
  left.resize(2, leftElement);
  valueA.left = left;
  ::ad::map::lane::GeoBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeoBorderTests, comparisonOperatorRightDiffers)
{
  ::ad::map::lane::GeoBorder valueA = mValue;
  ::ad::map::point::GeoEdge right;
  ::ad::map::point::GeoPoint rightElement;
  ::ad::map::point::Longitude rightElementLongitude(180);
  rightElement.longitude = rightElementLongitude;
  ::ad::map::point::Latitude rightElementLatitude(90);
  rightElement.latitude = rightElementLatitude;
  ::ad::map::point::Altitude rightElementAltitude(9000);
  rightElement.altitude = rightElementAltitude;
  right.resize(2, rightElement);
  valueA.right = right;
  ::ad::map::lane::GeoBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
