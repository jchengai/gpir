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
#include "ad/map/point/ENUPoint.hpp"

class ENUPointTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::ENUPoint value;
    ::ad::map::point::ENUCoordinate valueX(-16384);
    value.x = valueX;
    ::ad::map::point::ENUCoordinate valueY(-16384);
    value.y = valueY;
    ::ad::map::point::ENUCoordinate valueZ(-16384);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::map::point::ENUPoint mValue;
};

TEST_F(ENUPointTests, copyConstruction)
{
  ::ad::map::point::ENUPoint value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUPointTests, moveConstruction)
{
  ::ad::map::point::ENUPoint tmpValue(mValue);
  ::ad::map::point::ENUPoint value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUPointTests, copyAssignment)
{
  ::ad::map::point::ENUPoint value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUPointTests, moveAssignment)
{
  ::ad::map::point::ENUPoint tmpValue(mValue);
  ::ad::map::point::ENUPoint value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUPointTests, comparisonOperatorEqual)
{
  ::ad::map::point::ENUPoint valueA = mValue;
  ::ad::map::point::ENUPoint valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ENUPointTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ENUPointTests, comparisonOperatorXDiffers)
{
  ::ad::map::point::ENUPoint valueA = mValue;
  ::ad::map::point::ENUCoordinate x(16384);
  valueA.x = x;
  ::ad::map::point::ENUPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUPointTests, comparisonOperatorYDiffers)
{
  ::ad::map::point::ENUPoint valueA = mValue;
  ::ad::map::point::ENUCoordinate y(16384);
  valueA.y = y;
  ::ad::map::point::ENUPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUPointTests, comparisonOperatorZDiffers)
{
  ::ad::map::point::ENUPoint valueA = mValue;
  ::ad::map::point::ENUCoordinate z(16384);
  valueA.z = z;
  ::ad::map::point::ENUPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
