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
#include "ad/physics/AngularVelocity3D.hpp"

class AngularVelocity3DTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::AngularVelocity3D value;
    ::ad::physics::AngularVelocity valueX(-100.);
    value.x = valueX;
    ::ad::physics::AngularVelocity valueY(-100.);
    value.y = valueY;
    ::ad::physics::AngularVelocity valueZ(-100.);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::physics::AngularVelocity3D mValue;
};

TEST_F(AngularVelocity3DTests, copyConstruction)
{
  ::ad::physics::AngularVelocity3D value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AngularVelocity3DTests, moveConstruction)
{
  ::ad::physics::AngularVelocity3D tmpValue(mValue);
  ::ad::physics::AngularVelocity3D value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(AngularVelocity3DTests, copyAssignment)
{
  ::ad::physics::AngularVelocity3D value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(AngularVelocity3DTests, moveAssignment)
{
  ::ad::physics::AngularVelocity3D tmpValue(mValue);
  ::ad::physics::AngularVelocity3D value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AngularVelocity3DTests, comparisonOperatorEqual)
{
  ::ad::physics::AngularVelocity3D valueA = mValue;
  ::ad::physics::AngularVelocity3D valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(AngularVelocity3DTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(AngularVelocity3DTests, comparisonOperatorXDiffers)
{
  ::ad::physics::AngularVelocity3D valueA = mValue;
  ::ad::physics::AngularVelocity x(100.);
  valueA.x = x;
  ::ad::physics::AngularVelocity3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(AngularVelocity3DTests, comparisonOperatorYDiffers)
{
  ::ad::physics::AngularVelocity3D valueA = mValue;
  ::ad::physics::AngularVelocity y(100.);
  valueA.y = y;
  ::ad::physics::AngularVelocity3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(AngularVelocity3DTests, comparisonOperatorZDiffers)
{
  ::ad::physics::AngularVelocity3D valueA = mValue;
  ::ad::physics::AngularVelocity z(100.);
  valueA.z = z;
  ::ad::physics::AngularVelocity3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
