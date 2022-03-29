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
#include "ad/map/lane/ECEFBorder.hpp"

class ECEFBorderTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::lane::ECEFBorder value;
    ::ad::map::point::ECEFEdge valueLeft;
    ::ad::map::point::ECEFPoint valueLeftElement;
    ::ad::map::point::ECEFCoordinate valueLeftElementX(-6400000);
    valueLeftElement.x = valueLeftElementX;
    ::ad::map::point::ECEFCoordinate valueLeftElementY(-6400000);
    valueLeftElement.y = valueLeftElementY;
    ::ad::map::point::ECEFCoordinate valueLeftElementZ(-6400000);
    valueLeftElement.z = valueLeftElementZ;
    valueLeft.resize(1, valueLeftElement);
    value.left = valueLeft;
    ::ad::map::point::ECEFEdge valueRight;
    ::ad::map::point::ECEFPoint valueRightElement;
    ::ad::map::point::ECEFCoordinate valueRightElementX(-6400000);
    valueRightElement.x = valueRightElementX;
    ::ad::map::point::ECEFCoordinate valueRightElementY(-6400000);
    valueRightElement.y = valueRightElementY;
    ::ad::map::point::ECEFCoordinate valueRightElementZ(-6400000);
    valueRightElement.z = valueRightElementZ;
    valueRight.resize(1, valueRightElement);
    value.right = valueRight;
    mValue = value;
  }

  ::ad::map::lane::ECEFBorder mValue;
};

TEST_F(ECEFBorderTests, copyConstruction)
{
  ::ad::map::lane::ECEFBorder value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFBorderTests, moveConstruction)
{
  ::ad::map::lane::ECEFBorder tmpValue(mValue);
  ::ad::map::lane::ECEFBorder value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFBorderTests, copyAssignment)
{
  ::ad::map::lane::ECEFBorder value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFBorderTests, moveAssignment)
{
  ::ad::map::lane::ECEFBorder tmpValue(mValue);
  ::ad::map::lane::ECEFBorder value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ECEFBorderTests, comparisonOperatorEqual)
{
  ::ad::map::lane::ECEFBorder valueA = mValue;
  ::ad::map::lane::ECEFBorder valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ECEFBorderTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ECEFBorderTests, comparisonOperatorLeftDiffers)
{
  ::ad::map::lane::ECEFBorder valueA = mValue;
  ::ad::map::point::ECEFEdge left;
  ::ad::map::point::ECEFPoint leftElement;
  ::ad::map::point::ECEFCoordinate leftElementX(6400000);
  leftElement.x = leftElementX;
  ::ad::map::point::ECEFCoordinate leftElementY(6400000);
  leftElement.y = leftElementY;
  ::ad::map::point::ECEFCoordinate leftElementZ(6400000);
  leftElement.z = leftElementZ;
  left.resize(2, leftElement);
  valueA.left = left;
  ::ad::map::lane::ECEFBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ECEFBorderTests, comparisonOperatorRightDiffers)
{
  ::ad::map::lane::ECEFBorder valueA = mValue;
  ::ad::map::point::ECEFEdge right;
  ::ad::map::point::ECEFPoint rightElement;
  ::ad::map::point::ECEFCoordinate rightElementX(6400000);
  rightElement.x = rightElementX;
  ::ad::map::point::ECEFCoordinate rightElementY(6400000);
  rightElement.y = rightElementY;
  ::ad::map::point::ECEFCoordinate rightElementZ(6400000);
  rightElement.z = rightElementZ;
  right.resize(2, rightElement);
  valueA.right = right;
  ::ad::map::lane::ECEFBorder valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
