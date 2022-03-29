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
#include "ad/map/point/ECEFHeading.hpp"

class ECEFHeadingTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::ECEFHeading value;
    ::ad::map::point::ECEFCoordinate valueX(-6400000);
    value.x = valueX;
    ::ad::map::point::ECEFCoordinate valueY(-6400000);
    value.y = valueY;
    ::ad::map::point::ECEFCoordinate valueZ(-6400000);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::map::point::ECEFHeading mValue;
};

TEST_F(ECEFHeadingTests, copyConstruction)
{
  ::ad::map::point::ECEFHeading value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFHeadingTests, moveConstruction)
{
  ::ad::map::point::ECEFHeading tmpValue(mValue);
  ::ad::map::point::ECEFHeading value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFHeadingTests, copyAssignment)
{
  ::ad::map::point::ECEFHeading value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFHeadingTests, moveAssignment)
{
  ::ad::map::point::ECEFHeading tmpValue(mValue);
  ::ad::map::point::ECEFHeading value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFHeadingTests, comparisonOperatorEqual)
{
  ::ad::map::point::ECEFHeading valueA = mValue;
  ::ad::map::point::ECEFHeading valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ECEFHeadingTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ECEFHeadingTests, comparisonOperatorXDiffers)
{
  ::ad::map::point::ECEFHeading valueA = mValue;
  ::ad::map::point::ECEFCoordinate x(6400000);
  valueA.x = x;
  ::ad::map::point::ECEFHeading valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ECEFHeadingTests, comparisonOperatorYDiffers)
{
  ::ad::map::point::ECEFHeading valueA = mValue;
  ::ad::map::point::ECEFCoordinate y(6400000);
  valueA.y = y;
  ::ad::map::point::ECEFHeading valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ECEFHeadingTests, comparisonOperatorZDiffers)
{
  ::ad::map::point::ECEFHeading valueA = mValue;
  ::ad::map::point::ECEFCoordinate z(6400000);
  valueA.z = z;
  ::ad::map::point::ECEFHeading valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
