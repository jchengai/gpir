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
#include "ad/physics/AccelerationRange.hpp"

class AccelerationRangeTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::AccelerationRange value;
    ::ad::physics::Acceleration valueMinimum(-1e2);
    value.minimum = valueMinimum;
    ::ad::physics::Acceleration valueMaximum(-1e2);
    value.maximum = valueMaximum;
    value.maximum = value.minimum;
    value.minimum = value.maximum;
    mValue = value;
  }

  ::ad::physics::AccelerationRange mValue;
};

TEST_F(AccelerationRangeTests, copyConstruction)
{
  ::ad::physics::AccelerationRange value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AccelerationRangeTests, moveConstruction)
{
  ::ad::physics::AccelerationRange tmpValue(mValue);
  ::ad::physics::AccelerationRange value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(AccelerationRangeTests, copyAssignment)
{
  ::ad::physics::AccelerationRange value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(AccelerationRangeTests, moveAssignment)
{
  ::ad::physics::AccelerationRange tmpValue(mValue);
  ::ad::physics::AccelerationRange value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AccelerationRangeTests, comparisonOperatorEqual)
{
  ::ad::physics::AccelerationRange valueA = mValue;
  ::ad::physics::AccelerationRange valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(AccelerationRangeTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(AccelerationRangeTests, comparisonOperatorMinimumDiffers)
{
  ::ad::physics::AccelerationRange valueA = mValue;
  ::ad::physics::Acceleration minimum(1e2);
  valueA.minimum = minimum;
  ::ad::physics::AccelerationRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(AccelerationRangeTests, comparisonOperatorMaximumDiffers)
{
  ::ad::physics::AccelerationRange valueA = mValue;
  ::ad::physics::Acceleration maximum(1e2);
  valueA.maximum = maximum;
  ::ad::physics::AccelerationRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
