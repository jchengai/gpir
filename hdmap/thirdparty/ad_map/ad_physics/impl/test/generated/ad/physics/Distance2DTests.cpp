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
#include "ad/physics/Distance2D.hpp"

class Distance2DTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::Distance2D value;
    ::ad::physics::Distance valueX(-1e9);
    value.x = valueX;
    ::ad::physics::Distance valueY(-1e9);
    value.y = valueY;
    mValue = value;
  }

  ::ad::physics::Distance2D mValue;
};

TEST_F(Distance2DTests, copyConstruction)
{
  ::ad::physics::Distance2D value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(Distance2DTests, moveConstruction)
{
  ::ad::physics::Distance2D tmpValue(mValue);
  ::ad::physics::Distance2D value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(Distance2DTests, copyAssignment)
{
  ::ad::physics::Distance2D value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(Distance2DTests, moveAssignment)
{
  ::ad::physics::Distance2D tmpValue(mValue);
  ::ad::physics::Distance2D value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(Distance2DTests, comparisonOperatorEqual)
{
  ::ad::physics::Distance2D valueA = mValue;
  ::ad::physics::Distance2D valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(Distance2DTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(Distance2DTests, comparisonOperatorXDiffers)
{
  ::ad::physics::Distance2D valueA = mValue;
  ::ad::physics::Distance x(1e9);
  valueA.x = x;
  ::ad::physics::Distance2D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(Distance2DTests, comparisonOperatorYDiffers)
{
  ::ad::physics::Distance2D valueA = mValue;
  ::ad::physics::Distance y(1e9);
  valueA.y = y;
  ::ad::physics::Distance2D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
