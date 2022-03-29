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
#include "ad/map/match/ENUObjectPosition.hpp"

class ENUObjectPositionTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::ENUObjectPosition value;
    ::ad::map::point::ENUPoint valueCenterPoint;
    ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
    valueCenterPoint.x = valueCenterPointX;
    ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
    valueCenterPoint.y = valueCenterPointY;
    ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
    valueCenterPoint.z = valueCenterPointZ;
    value.centerPoint = valueCenterPoint;
    ::ad::map::point::ENUHeading valueHeading(-3.141592655);
    value.heading = valueHeading;
    ::ad::map::point::GeoPoint valueEnuReferencePoint;
    ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
    valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
    ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
    valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
    ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
    valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
    value.enuReferencePoint = valueEnuReferencePoint;
    ::ad::physics::Dimension3D valueDimension;
    ::ad::physics::Distance valueDimensionLength(-1e9);
    valueDimension.length = valueDimensionLength;
    ::ad::physics::Distance valueDimensionWidth(-1e9);
    valueDimension.width = valueDimensionWidth;
    ::ad::physics::Distance valueDimensionHeight(-1e9);
    valueDimension.height = valueDimensionHeight;
    value.dimension = valueDimension;
    mValue = value;
  }

  ::ad::map::match::ENUObjectPosition mValue;
};

TEST_F(ENUObjectPositionTests, copyConstruction)
{
  ::ad::map::match::ENUObjectPosition value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUObjectPositionTests, moveConstruction)
{
  ::ad::map::match::ENUObjectPosition tmpValue(mValue);
  ::ad::map::match::ENUObjectPosition value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUObjectPositionTests, copyAssignment)
{
  ::ad::map::match::ENUObjectPosition value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUObjectPositionTests, moveAssignment)
{
  ::ad::map::match::ENUObjectPosition tmpValue(mValue);
  ::ad::map::match::ENUObjectPosition value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUObjectPositionTests, comparisonOperatorEqual)
{
  ::ad::map::match::ENUObjectPosition valueA = mValue;
  ::ad::map::match::ENUObjectPosition valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ENUObjectPositionTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ENUObjectPositionTests, comparisonOperatorCenterPointDiffers)
{
  ::ad::map::match::ENUObjectPosition valueA = mValue;
  ::ad::map::point::ENUPoint centerPoint;
  ::ad::map::point::ENUCoordinate centerPointX(16384);
  centerPoint.x = centerPointX;
  ::ad::map::point::ENUCoordinate centerPointY(16384);
  centerPoint.y = centerPointY;
  ::ad::map::point::ENUCoordinate centerPointZ(16384);
  centerPoint.z = centerPointZ;
  valueA.centerPoint = centerPoint;
  ::ad::map::match::ENUObjectPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUObjectPositionTests, comparisonOperatorHeadingDiffers)
{
  ::ad::map::match::ENUObjectPosition valueA = mValue;
  ::ad::map::point::ENUHeading heading(3.141592655);
  valueA.heading = heading;
  ::ad::map::match::ENUObjectPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUObjectPositionTests, comparisonOperatorEnuReferencePointDiffers)
{
  ::ad::map::match::ENUObjectPosition valueA = mValue;
  ::ad::map::point::GeoPoint enuReferencePoint;
  ::ad::map::point::Longitude enuReferencePointLongitude(180);
  enuReferencePoint.longitude = enuReferencePointLongitude;
  ::ad::map::point::Latitude enuReferencePointLatitude(90);
  enuReferencePoint.latitude = enuReferencePointLatitude;
  ::ad::map::point::Altitude enuReferencePointAltitude(9000);
  enuReferencePoint.altitude = enuReferencePointAltitude;
  valueA.enuReferencePoint = enuReferencePoint;
  ::ad::map::match::ENUObjectPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUObjectPositionTests, comparisonOperatorDimensionDiffers)
{
  ::ad::map::match::ENUObjectPosition valueA = mValue;
  ::ad::physics::Dimension3D dimension;
  ::ad::physics::Distance dimensionLength(1e9);
  dimension.length = dimensionLength;
  ::ad::physics::Distance dimensionWidth(1e9);
  dimension.width = dimensionWidth;
  ::ad::physics::Distance dimensionHeight(1e9);
  dimension.height = dimensionHeight;
  valueA.dimension = dimension;
  ::ad::map::match::ENUObjectPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
