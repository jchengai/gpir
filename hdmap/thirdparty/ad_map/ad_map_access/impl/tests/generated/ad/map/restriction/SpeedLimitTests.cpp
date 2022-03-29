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
#include "ad/map/restriction/SpeedLimit.hpp"

class SpeedLimitTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::restriction::SpeedLimit value;
    ::ad::physics::Speed valueSpeedLimit(-100.);
    value.speedLimit = valueSpeedLimit;
    ::ad::physics::ParametricRange valueLanePiece;
    ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
    valueLanePiece.minimum = valueLanePieceMinimum;
    ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
    valueLanePiece.maximum = valueLanePieceMaximum;
    valueLanePiece.maximum = valueLanePiece.minimum;
    valueLanePiece.minimum = valueLanePiece.maximum;
    value.lanePiece = valueLanePiece;
    mValue = value;
  }

  ::ad::map::restriction::SpeedLimit mValue;
};

TEST_F(SpeedLimitTests, copyConstruction)
{
  ::ad::map::restriction::SpeedLimit value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedLimitTests, moveConstruction)
{
  ::ad::map::restriction::SpeedLimit tmpValue(mValue);
  ::ad::map::restriction::SpeedLimit value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedLimitTests, copyAssignment)
{
  ::ad::map::restriction::SpeedLimit value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedLimitTests, moveAssignment)
{
  ::ad::map::restriction::SpeedLimit tmpValue(mValue);
  ::ad::map::restriction::SpeedLimit value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(SpeedLimitTests, comparisonOperatorEqual)
{
  ::ad::map::restriction::SpeedLimit valueA = mValue;
  ::ad::map::restriction::SpeedLimit valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(SpeedLimitTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(SpeedLimitTests, comparisonOperatorSpeedLimitDiffers)
{
  ::ad::map::restriction::SpeedLimit valueA = mValue;
  ::ad::physics::Speed speedLimit(100.);
  valueA.speedLimit = speedLimit;
  ::ad::map::restriction::SpeedLimit valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(SpeedLimitTests, comparisonOperatorLanePieceDiffers)
{
  ::ad::map::restriction::SpeedLimit valueA = mValue;
  ::ad::physics::ParametricRange lanePiece;
  ::ad::physics::ParametricValue lanePieceMinimum(1.);
  lanePiece.minimum = lanePieceMinimum;
  ::ad::physics::ParametricValue lanePieceMaximum(1.);
  lanePiece.maximum = lanePieceMaximum;
  lanePiece.maximum = lanePiece.minimum;
  lanePiece.minimum = lanePiece.maximum;
  valueA.lanePiece = lanePiece;
  ::ad::map::restriction::SpeedLimit valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
