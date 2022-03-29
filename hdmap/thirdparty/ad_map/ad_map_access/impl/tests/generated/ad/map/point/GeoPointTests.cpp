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
#include "ad/map/point/GeoPoint.hpp"

class GeoPointTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::GeoPoint value;
    ::ad::map::point::Longitude valueLongitude(-180);
    value.longitude = valueLongitude;
    ::ad::map::point::Latitude valueLatitude(-90);
    value.latitude = valueLatitude;
    ::ad::map::point::Altitude valueAltitude(-11000);
    value.altitude = valueAltitude;
    mValue = value;
  }

  ::ad::map::point::GeoPoint mValue;
};

TEST_F(GeoPointTests, copyConstruction)
{
  ::ad::map::point::GeoPoint value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoPointTests, moveConstruction)
{
  ::ad::map::point::GeoPoint tmpValue(mValue);
  ::ad::map::point::GeoPoint value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoPointTests, copyAssignment)
{
  ::ad::map::point::GeoPoint value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoPointTests, moveAssignment)
{
  ::ad::map::point::GeoPoint tmpValue(mValue);
  ::ad::map::point::GeoPoint value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeoPointTests, comparisonOperatorEqual)
{
  ::ad::map::point::GeoPoint valueA = mValue;
  ::ad::map::point::GeoPoint valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(GeoPointTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(GeoPointTests, comparisonOperatorLongitudeDiffers)
{
  ::ad::map::point::GeoPoint valueA = mValue;
  ::ad::map::point::Longitude longitude(180);
  valueA.longitude = longitude;
  ::ad::map::point::GeoPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeoPointTests, comparisonOperatorLatitudeDiffers)
{
  ::ad::map::point::GeoPoint valueA = mValue;
  ::ad::map::point::Latitude latitude(90);
  valueA.latitude = latitude;
  ::ad::map::point::GeoPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeoPointTests, comparisonOperatorAltitudeDiffers)
{
  ::ad::map::point::GeoPoint valueA = mValue;
  ::ad::map::point::Altitude altitude(9000);
  valueA.altitude = altitude;
  ::ad::map::point::GeoPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
