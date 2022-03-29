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
#include "ad/physics/Velocity.hpp"

class VelocityTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::Velocity value;
    ::ad::physics::Speed valueX(-100.);
    value.x = valueX;
    ::ad::physics::Speed valueY(-100.);
    value.y = valueY;
    ::ad::physics::Speed valueZ(-100.);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::physics::Velocity mValue;
};

TEST_F(VelocityTests, copyConstruction)
{
  ::ad::physics::Velocity value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(VelocityTests, moveConstruction)
{
  ::ad::physics::Velocity tmpValue(mValue);
  ::ad::physics::Velocity value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(VelocityTests, copyAssignment)
{
  ::ad::physics::Velocity value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(VelocityTests, moveAssignment)
{
  ::ad::physics::Velocity tmpValue(mValue);
  ::ad::physics::Velocity value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(VelocityTests, comparisonOperatorEqual)
{
  ::ad::physics::Velocity valueA = mValue;
  ::ad::physics::Velocity valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(VelocityTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(VelocityTests, comparisonOperatorXDiffers)
{
  ::ad::physics::Velocity valueA = mValue;
  ::ad::physics::Speed x(100.);
  valueA.x = x;
  ::ad::physics::Velocity valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VelocityTests, comparisonOperatorYDiffers)
{
  ::ad::physics::Velocity valueA = mValue;
  ::ad::physics::Speed y(100.);
  valueA.y = y;
  ::ad::physics::Velocity valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VelocityTests, comparisonOperatorZDiffers)
{
  ::ad::physics::Velocity valueA = mValue;
  ::ad::physics::Speed z(100.);
  valueA.z = z;
  ::ad::physics::Velocity valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
