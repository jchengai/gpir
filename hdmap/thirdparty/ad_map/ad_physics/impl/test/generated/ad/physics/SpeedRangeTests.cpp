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
#include "ad/physics/SpeedRange.hpp"

class SpeedRangeTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::SpeedRange value;
    ::ad::physics::Speed valueMinimum(-100.);
    value.minimum = valueMinimum;
    ::ad::physics::Speed valueMaximum(-100.);
    value.maximum = valueMaximum;
    value.maximum = value.minimum;
    value.minimum = value.maximum;
    mValue = value;
  }

  ::ad::physics::SpeedRange mValue;
};

TEST_F(SpeedRangeTests, copyConstruction)
{
  ::ad::physics::SpeedRange value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedRangeTests, moveConstruction)
{
  ::ad::physics::SpeedRange tmpValue(mValue);
  ::ad::physics::SpeedRange value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedRangeTests, copyAssignment)
{
  ::ad::physics::SpeedRange value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedRangeTests, moveAssignment)
{
  ::ad::physics::SpeedRange tmpValue(mValue);
  ::ad::physics::SpeedRange value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedRangeTests, comparisonOperatorEqual)
{
  ::ad::physics::SpeedRange valueA = mValue;
  ::ad::physics::SpeedRange valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(SpeedRangeTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(SpeedRangeTests, comparisonOperatorMinimumDiffers)
{
  ::ad::physics::SpeedRange valueA = mValue;
  ::ad::physics::Speed minimum(100.);
  valueA.minimum = minimum;
  ::ad::physics::SpeedRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(SpeedRangeTests, comparisonOperatorMaximumDiffers)
{
  ::ad::physics::SpeedRange valueA = mValue;
  ::ad::physics::Speed maximum(100.);
  valueA.maximum = maximum;
  ::ad::physics::SpeedRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
