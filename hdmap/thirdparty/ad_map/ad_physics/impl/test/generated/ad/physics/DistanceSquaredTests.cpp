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
#include "ad/physics/Distance.hpp"
#include "ad/physics/DistanceSquared.hpp"

TEST(DistanceSquaredTests, defaultConstructionIsInvalid)
{
  ::ad::physics::DistanceSquared value;
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, minIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(-1e18, ::ad::physics::DistanceSquared::cMinValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DistanceSquared::cMinValue,
                   static_cast<double>(::ad::physics::DistanceSquared::getMin()));
}

TEST(DistanceSquaredTests, maxIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(1e18, ::ad::physics::DistanceSquared::cMaxValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DistanceSquared::cMaxValue,
                   static_cast<double>(::ad::physics::DistanceSquared::getMax()));
}

TEST(DistanceSquaredTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::DistanceSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-6, ::ad::physics::DistanceSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DistanceSquared::cPrecisionValue,
                   static_cast<double>(::ad::physics::DistanceSquared::getPrecision()));
}

TEST(DistanceSquaredTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::DistanceSquared::getMin().isValid());
}

TEST(DistanceSquaredTests, aboveMinIsValid)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(DistanceSquaredTests, belowMinIsInvalid)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::DistanceSquared::getMax().isValid());
}

TEST(DistanceSquaredTests, aboveMaxIsInvalid)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, belowMaxIsValid)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_DISTANCESQUARED_THROWS_EXCEPTION == 1)
TEST(DistanceSquaredTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::DistanceSquared value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(DistanceSquaredTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::DistanceSquared value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(DistanceSquaredTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::DistanceSquared value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(DistanceSquaredTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DistanceSquared::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DistanceSquared>::lowest()));
}

TEST(DistanceSquaredTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DistanceSquared::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DistanceSquared>::max()));
}

TEST(DistanceSquaredTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DistanceSquared::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DistanceSquared>::epsilon()));
}

TEST(DistanceSquaredTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::DistanceSquared(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::DistanceSquared(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::physics::DistanceSquared::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DistanceSquared::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DistanceSquared::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::DistanceSquared(-::ad::physics::DistanceSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DistanceSquared::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DistanceSquared::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::DistanceSquared(-::ad::physics::DistanceSquared::cMaxValue))));
}

TEST(DistanceSquaredTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::DistanceSquared::cMinValue;
  ::ad::physics::DistanceSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(DistanceSquaredTests, copyConstructionFromValidValue)
{
  ::ad::physics::DistanceSquared const validValue(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(DistanceSquaredTests, moveConstructionFromValidValue)
{
  ::ad::physics::DistanceSquared validValue(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::DistanceSquared::cMinValue, static_cast<double>(value));
}

TEST(DistanceSquaredTests, assignmentFromValidValue)
{
  ::ad::physics::DistanceSquared const validValue(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(DistanceSquaredTests, moveAssignmentFromValidValue)
{
  ::ad::physics::DistanceSquared validValue(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::DistanceSquared::cMinValue, static_cast<double>(value));
}

TEST(DistanceSquaredTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::DistanceSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::DistanceSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::DistanceSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, assignmentFromInvalidValue)
{
  ::ad::physics::DistanceSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::DistanceSquared value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(DistanceSquaredTests, selfAssignment)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(DistanceSquaredTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::DistanceSquared value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_DISTANCESQUARED_THROWS_EXCEPTION == 1)
TEST(DistanceSquaredTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::DistanceSquared const value(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared const invalidValue;

  EXPECT_THROW((void)(invalidValue == value), std::out_of_range);
  EXPECT_THROW((void)(value == invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue != value), std::out_of_range);
  EXPECT_THROW((void)(value != invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue > value), std::out_of_range);
  EXPECT_THROW((void)(value > invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue < value), std::out_of_range);
  EXPECT_THROW((void)(value < invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue >= value), std::out_of_range);
  EXPECT_THROW((void)(value >= invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue <= value), std::out_of_range);
  EXPECT_THROW((void)(value <= invalidValue), std::out_of_range);
}

TEST(DistanceSquaredTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::DistanceSquared const minimalValue(::ad::physics::DistanceSquared::cMinValue);
  ::ad::physics::DistanceSquared const maximalValue(::ad::physics::DistanceSquared::cMaxValue);
  ::ad::physics::DistanceSquared const invalidValue;
  ::ad::physics::DistanceSquared calculationValue;

  //  operator+(::ad::physics::DistanceSquared)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::DistanceSquared)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::DistanceSquared)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::DistanceSquared)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue -= minimalValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= invalidValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= maximalValue, std::out_of_range);

  //  operator*(double)
  EXPECT_THROW(invalidValue * static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue * static_cast<double>(maximalValue), std::out_of_range);

  //  std::sqrt()
  EXPECT_THROW(std::sqrt(invalidValue), std::out_of_range);

  //  operator/(double)
  EXPECT_THROW(invalidValue / static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue / static_cast<double>(invalidValue), std::out_of_range);
  EXPECT_THROW(maximalValue / 0.0, std::out_of_range);
  EXPECT_THROW(maximalValue / 0.5, std::out_of_range);

  //  operator/(::ad::physics::DistanceSquared)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::DistanceSquared(0.0), std::out_of_range);

  //  operator-()
  EXPECT_THROW(-invalidValue, std::out_of_range);
  if (std::fabs(static_cast<double>(maximalValue)) > std::fabs(static_cast<double>(minimalValue)))
  {
    EXPECT_THROW(-maximalValue, std::out_of_range);
  }
  else if (std::fabs(static_cast<double>(maximalValue)) < std::fabs(static_cast<double>(minimalValue)))
  {
    EXPECT_THROW(-minimalValue, std::out_of_range);
  }
  else
  {
    // impossible to trigger invalid result while operand is valid
  }
}
#endif

TEST(DistanceSquaredTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::DistanceSquared::cPrecisionValue * 10.;
  ::ad::physics::DistanceSquared value;
  if (::ad::physics::DistanceSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::DistanceSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::DistanceSquared(precisionValueTimesTen);
  }
  ::ad::physics::DistanceSquared const sameValue = value;
  ::ad::physics::DistanceSquared const slightlyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::DistanceSquared::cPrecisionValue * 0.9);
  ::ad::physics::DistanceSquared const slightlySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::DistanceSquared::cPrecisionValue * 0.9);
  ::ad::physics::DistanceSquared const actuallyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::DistanceSquared::cPrecisionValue * 1.1);
  ::ad::physics::DistanceSquared const actuallySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::DistanceSquared::cPrecisionValue * 1.1);

  // operator ==
  EXPECT_TRUE(value == sameValue);
  EXPECT_TRUE(value == slightlyBiggerValue);
  EXPECT_TRUE(value == slightlySmallerValue);
  EXPECT_FALSE(value == actuallyBiggerValue);
  EXPECT_FALSE(value == actuallySmallerValue);

  // operator !=
  EXPECT_FALSE(value != sameValue);
  EXPECT_FALSE(value != slightlyBiggerValue);
  EXPECT_FALSE(value != slightlySmallerValue);
  EXPECT_TRUE(value != actuallyBiggerValue);
  EXPECT_TRUE(value != actuallySmallerValue);

  // operator >
  EXPECT_FALSE(value > value);
  EXPECT_FALSE(slightlyBiggerValue > value);
  EXPECT_TRUE(actuallyBiggerValue > value);

  // operator >=
  EXPECT_FALSE(actuallySmallerValue >= value);
  EXPECT_TRUE(slightlySmallerValue >= value);
  EXPECT_TRUE(value >= value);
  EXPECT_TRUE(slightlyBiggerValue >= value);
  EXPECT_TRUE(actuallyBiggerValue >= value);

  // operator <
  EXPECT_FALSE(value < value);
  EXPECT_FALSE(slightlySmallerValue < value);
  EXPECT_TRUE(actuallySmallerValue < value);

  // operator <=
  EXPECT_FALSE(actuallyBiggerValue <= value);
  EXPECT_TRUE(slightlyBiggerValue <= value);
  EXPECT_TRUE(value <= value);
  EXPECT_TRUE(slightlySmallerValue <= value);
  EXPECT_TRUE(actuallySmallerValue <= value);
}

TEST(DistanceSquaredTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::DistanceSquared::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::DistanceSquared::cPrecisionValue * 10.;
  ::ad::physics::DistanceSquared value;
  if (::ad::physics::DistanceSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::DistanceSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::DistanceSquared(::ad::physics::DistanceSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::DistanceSquared(precisionValueTimesTen);
  }

  ::ad::physics::DistanceSquared result;

  //  operator+(::ad::physics::DistanceSquared)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::DistanceSquared)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::DistanceSquared)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::DistanceSquared)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  std::sqrt()
  ::ad::physics::Distance const squareRootResult = std::sqrt(value);
  EXPECT_NEAR(std::sqrt(static_cast<double>(value)), static_cast<double>(squareRootResult), cDoubleNear);

  //  operator*(double, ::ad::physics::DistanceSquared)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::DistanceSquared)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::DistanceSquared::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::DistanceSquared::cMaxValue))
  {
    result = -value;
  }
  else
  {
    // not clear on how to trigger valid result if such small value is not working
  }
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
