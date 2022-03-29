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
#include "ad/map/config/PointOfInterest.hpp"

class PointOfInterestTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::config::PointOfInterest value;
    ::ad::map::point::GeoPoint valueGeoPoint;
    ::ad::map::point::Longitude valueGeoPointLongitude(-180);
    valueGeoPoint.longitude = valueGeoPointLongitude;
    ::ad::map::point::Latitude valueGeoPointLatitude(-90);
    valueGeoPoint.latitude = valueGeoPointLatitude;
    ::ad::map::point::Altitude valueGeoPointAltitude(-11000);
    valueGeoPoint.altitude = valueGeoPointAltitude;
    value.geoPoint = valueGeoPoint;
    std::string valueName{"min"};
    value.name = valueName;
    mValue = value;
  }

  ::ad::map::config::PointOfInterest mValue;
};

TEST_F(PointOfInterestTests, copyConstruction)
{
  ::ad::map::config::PointOfInterest value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(PointOfInterestTests, moveConstruction)
{
  ::ad::map::config::PointOfInterest tmpValue(mValue);
  ::ad::map::config::PointOfInterest value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(PointOfInterestTests, copyAssignment)
{
  ::ad::map::config::PointOfInterest value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(PointOfInterestTests, moveAssignment)
{
  ::ad::map::config::PointOfInterest tmpValue(mValue);
  ::ad::map::config::PointOfInterest value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(PointOfInterestTests, comparisonOperatorEqual)
{
  ::ad::map::config::PointOfInterest valueA = mValue;
  ::ad::map::config::PointOfInterest valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(PointOfInterestTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(PointOfInterestTests, comparisonOperatorGeoPointDiffers)
{
  ::ad::map::config::PointOfInterest valueA = mValue;
  ::ad::map::point::GeoPoint geoPoint;
  ::ad::map::point::Longitude geoPointLongitude(180);
  geoPoint.longitude = geoPointLongitude;
  ::ad::map::point::Latitude geoPointLatitude(90);
  geoPoint.latitude = geoPointLatitude;
  ::ad::map::point::Altitude geoPointAltitude(9000);
  geoPoint.altitude = geoPointAltitude;
  valueA.geoPoint = geoPoint;
  ::ad::map::config::PointOfInterest valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(PointOfInterestTests, comparisonOperatorNameDiffers)
{
  ::ad::map::config::PointOfInterest valueA = mValue;
  std::string name{"max"};
  valueA.name = name;
  ::ad::map::config::PointOfInterest valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
