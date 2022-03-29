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
#include "ad/map/point/ECEFPoint.hpp"

class ECEFPointTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::ECEFPoint value;
    ::ad::map::point::ECEFCoordinate valueX(-6400000);
    value.x = valueX;
    ::ad::map::point::ECEFCoordinate valueY(-6400000);
    value.y = valueY;
    ::ad::map::point::ECEFCoordinate valueZ(-6400000);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::map::point::ECEFPoint mValue;
};

TEST_F(ECEFPointTests, copyConstruction)
{
  ::ad::map::point::ECEFPoint value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFPointTests, moveConstruction)
{
  ::ad::map::point::ECEFPoint tmpValue(mValue);
  ::ad::map::point::ECEFPoint value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFPointTests, copyAssignment)
{
  ::ad::map::point::ECEFPoint value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFPointTests, moveAssignment)
{
  ::ad::map::point::ECEFPoint tmpValue(mValue);
  ::ad::map::point::ECEFPoint value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFPointTests, comparisonOperatorEqual)
{
  ::ad::map::point::ECEFPoint valueA = mValue;
  ::ad::map::point::ECEFPoint valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ECEFPointTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ECEFPointTests, comparisonOperatorXDiffers)
{
  ::ad::map::point::ECEFPoint valueA = mValue;
  ::ad::map::point::ECEFCoordinate x(6400000);
  valueA.x = x;
  ::ad::map::point::ECEFPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ECEFPointTests, comparisonOperatorYDiffers)
{
  ::ad::map::point::ECEFPoint valueA = mValue;
  ::ad::map::point::ECEFCoordinate y(6400000);
  valueA.y = y;
  ::ad::map::point::ECEFPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ECEFPointTests, comparisonOperatorZDiffers)
{
  ::ad::map::point::ECEFPoint valueA = mValue;
  ::ad::map::point::ECEFCoordinate z(6400000);
  valueA.z = z;
  ::ad::map::point::ECEFPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
