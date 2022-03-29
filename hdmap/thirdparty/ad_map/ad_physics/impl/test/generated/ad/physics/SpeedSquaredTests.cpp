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
#include "ad/physics/Speed.hpp"
#include "ad/physics/SpeedSquared.hpp"

TEST(SpeedSquaredTests, defaultConstructionIsInvalid)
{
  ::ad::physics::SpeedSquared value;
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, minIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(-1e6, ::ad::physics::SpeedSquared::cMinValue);
  EXPECT_DOUBLE_EQ(::ad::physics::SpeedSquared::cMinValue, static_cast<double>(::ad::physics::SpeedSquared::getMin()));
}

TEST(SpeedSquaredTests, maxIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(1e6, ::ad::physics::SpeedSquared::cMaxValue);
  EXPECT_DOUBLE_EQ(::ad::physics::SpeedSquared::cMaxValue, static_cast<double>(::ad::physics::SpeedSquared::getMax()));
}

TEST(SpeedSquaredTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::SpeedSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-6, ::ad::physics::SpeedSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::SpeedSquared::cPrecisionValue,
                   static_cast<double>(::ad::physics::SpeedSquared::getPrecision()));
}

TEST(SpeedSquaredTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::SpeedSquared::getMin().isValid());
}

TEST(SpeedSquaredTests, aboveMinIsValid)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(SpeedSquaredTests, belowMinIsInvalid)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::SpeedSquared::getMax().isValid());
}

TEST(SpeedSquaredTests, aboveMaxIsInvalid)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, belowMaxIsValid)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_SPEEDSQUARED_THROWS_EXCEPTION == 1)
TEST(SpeedSquaredTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::SpeedSquared value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(SpeedSquaredTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::SpeedSquared value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(SpeedSquaredTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::SpeedSquared value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(SpeedSquaredTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::SpeedSquared::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::SpeedSquared>::lowest()));
}

TEST(SpeedSquaredTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::SpeedSquared::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::SpeedSquared>::max()));
}

TEST(SpeedSquaredTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::SpeedSquared::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::SpeedSquared>::epsilon()));
}

TEST(SpeedSquaredTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::SpeedSquared(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::SpeedSquared(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::physics::SpeedSquared::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::SpeedSquared::cMinValue),
                   static_cast<double>(std::fabs(::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::SpeedSquared::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::SpeedSquared(-::ad::physics::SpeedSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::SpeedSquared::cMaxValue),
                   static_cast<double>(std::fabs(::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::SpeedSquared::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::SpeedSquared(-::ad::physics::SpeedSquared::cMaxValue))));
}

TEST(SpeedSquaredTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::SpeedSquared::cMinValue;
  ::ad::physics::SpeedSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(SpeedSquaredTests, copyConstructionFromValidValue)
{
  ::ad::physics::SpeedSquared const validValue(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(SpeedSquaredTests, moveConstructionFromValidValue)
{
  ::ad::physics::SpeedSquared validValue(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::SpeedSquared::cMinValue, static_cast<double>(value));
}

TEST(SpeedSquaredTests, assignmentFromValidValue)
{
  ::ad::physics::SpeedSquared const validValue(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(SpeedSquaredTests, moveAssignmentFromValidValue)
{
  ::ad::physics::SpeedSquared validValue(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::SpeedSquared::cMinValue, static_cast<double>(value));
}

TEST(SpeedSquaredTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::SpeedSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::SpeedSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::SpeedSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, assignmentFromInvalidValue)
{
  ::ad::physics::SpeedSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::SpeedSquared value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(SpeedSquaredTests, selfAssignment)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(SpeedSquaredTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::SpeedSquared value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_SPEEDSQUARED_THROWS_EXCEPTION == 1)
TEST(SpeedSquaredTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::SpeedSquared const value(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared const invalidValue;

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

TEST(SpeedSquaredTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::SpeedSquared const minimalValue(::ad::physics::SpeedSquared::cMinValue);
  ::ad::physics::SpeedSquared const maximalValue(::ad::physics::SpeedSquared::cMaxValue);
  ::ad::physics::SpeedSquared const invalidValue;
  ::ad::physics::SpeedSquared calculationValue;

  //  operator+(::ad::physics::SpeedSquared)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::SpeedSquared)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::SpeedSquared)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::SpeedSquared)
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

  //  operator/(::ad::physics::SpeedSquared)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::SpeedSquared(0.0), std::out_of_range);

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

TEST(SpeedSquaredTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::SpeedSquared::cPrecisionValue * 10.;
  ::ad::physics::SpeedSquared value;
  if (::ad::physics::SpeedSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::SpeedSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::SpeedSquared(precisionValueTimesTen);
  }
  ::ad::physics::SpeedSquared const sameValue = value;
  ::ad::physics::SpeedSquared const slightlyBiggerValue(static_cast<double>(value)
                                                        + ::ad::physics::SpeedSquared::cPrecisionValue * 0.9);
  ::ad::physics::SpeedSquared const slightlySmallerValue(static_cast<double>(value)
                                                         - ::ad::physics::SpeedSquared::cPrecisionValue * 0.9);
  ::ad::physics::SpeedSquared const actuallyBiggerValue(static_cast<double>(value)
                                                        + ::ad::physics::SpeedSquared::cPrecisionValue * 1.1);
  ::ad::physics::SpeedSquared const actuallySmallerValue(static_cast<double>(value)
                                                         - ::ad::physics::SpeedSquared::cPrecisionValue * 1.1);

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

TEST(SpeedSquaredTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::SpeedSquared::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::SpeedSquared::cPrecisionValue * 10.;
  ::ad::physics::SpeedSquared value;
  if (::ad::physics::SpeedSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::SpeedSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::SpeedSquared(::ad::physics::SpeedSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::SpeedSquared(precisionValueTimesTen);
  }

  ::ad::physics::SpeedSquared result;

  //  operator+(::ad::physics::SpeedSquared)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::SpeedSquared)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::SpeedSquared)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::SpeedSquared)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  std::sqrt()
  ::ad::physics::Speed const squareRootResult = std::sqrt(value);
  EXPECT_NEAR(std::sqrt(static_cast<double>(value)), static_cast<double>(squareRootResult), cDoubleNear);

  //  operator*(double, ::ad::physics::SpeedSquared)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::SpeedSquared)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::SpeedSquared::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::SpeedSquared::cMaxValue))
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
