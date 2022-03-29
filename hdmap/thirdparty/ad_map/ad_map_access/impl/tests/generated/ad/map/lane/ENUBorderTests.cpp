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
#include "ad/map/lane/ENUBorder.hpp"

class ENUBorderTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::lane::ENUBorder value;
    ::ad::map::point::ENUEdge valueLeft;
    ::ad::map::point::ENUPoint valueLeftElement;
    ::ad::map::point::ENUCoordinate valueLeftElementX(-16384);
    valueLeftElement.x = valueLeftElementX;
    ::ad::map::point::ENUCoordinate valueLeftElementY(-16384);
    valueLeftElement.y = valueLeftElementY;
    ::ad::map::point::ENUCoordinate valueLeftElementZ(-16384);
    valueLeftElement.z = valueLeftElementZ;
    valueLeft.resize(1, valueLeftElement);
    value.left = valueLeft;
    ::ad::map::point::ENUEdge valueRight;
    ::ad::map::point::ENUPoint valueRightElement;
    ::ad::map::point::ENUCoordinate valueRightElementX(-16384);
    valueRightElement.x = valueRightElementX;
    ::ad::map::point::ENUCoordinate valueRightElementY(-16384);
    valueRightElement.y = valueRightElementY;
    ::ad::map::point::ENUCoordinate valueRightElementZ(-16384);
    valueRightElement.z = valueRightElementZ;
    valueRight.resize(1, valueRightElement);
    value.right = valueRight;
    mValue = value;
  }

  ::ad::map::lane::ENUBorder mValue;
};

TEST_F(ENUBorderTests, copyConstruction)
{
  ::ad::map::lane::ENUBorder value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUBorderTests, moveConstruction)
{
  ::ad::map::lane::ENUBorder tmpValue(mValue);
  ::ad::map::lane::ENUBorder value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUBorderTests, copyAssignment)
{
  ::ad::map::lane::ENUBorder value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUBorderTests, moveAssignment)
{
  ::ad::map::lane::ENUBorder tmpValue(mValue);
  ::ad::map::lane::ENUBorder value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUBorderTests, comparisonOperatorEqual)
{
  ::ad::map::lane::ENUBorder valueA = mValue;
  ::ad::map::lane::ENUBorder valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ENUBorderTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ENUBorderTests, comparisonOperatorLeftDiffers)
{
  ::ad::map::lane::ENUBorder valueA = mValue;
  ::ad::map::point::ENUEdge left;
  ::ad::map::point::ENUPoint leftElement;
  ::ad::map::point::ENUCoordinate leftElementX(16384);
  leftElement.x = leftElementX;
  ::ad::map::point::ENUCoordinate leftElementY(16384);
  leftElement.y = leftElementY;
  ::ad::map::point::ENUCoordinate leftElementZ(16384);
  leftElement.z = leftElementZ;
  left.resize(2, leftElement);
  valueA.left = left;
  ::ad::map::lane::ENUBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUBorderTests, comparisonOperatorRightDiffers)
{
  ::ad::map::lane::ENUBorder valueA = mValue;
  ::ad::map::point::ENUEdge right;
  ::ad::map::point::ENUPoint rightElement;
  ::ad::map::point::ENUCoordinate rightElementX(16384);
  rightElement.x = rightElementX;
  ::ad::map::point::ENUCoordinate rightElementY(16384);
  rightElement.y = rightElementY;
  ::ad::map::point::ENUCoordinate rightElementZ(16384);
  rightElement.z = rightElementZ;
  right.resize(2, rightElement);
  valueA.right = right;
  ::ad::map::lane::ENUBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
