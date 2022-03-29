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
#include "ad/physics/Duration.hpp"
#include "ad/physics/DurationSquared.hpp"

TEST(DurationSquaredTests, defaultConstructionIsInvalid)
{
  ::ad::physics::DurationSquared value;
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, minIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(-1e12, ::ad::physics::DurationSquared::cMinValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DurationSquared::cMinValue,
                   static_cast<double>(::ad::physics::DurationSquared::getMin()));
}

TEST(DurationSquaredTests, maxIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(1e12, ::ad::physics::DurationSquared::cMaxValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DurationSquared::cMaxValue,
                   static_cast<double>(::ad::physics::DurationSquared::getMax()));
}

TEST(DurationSquaredTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::DurationSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-6, ::ad::physics::DurationSquared::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::DurationSquared::cPrecisionValue,
                   static_cast<double>(::ad::physics::DurationSquared::getPrecision()));
}

TEST(DurationSquaredTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::DurationSquared::getMin().isValid());
}

TEST(DurationSquaredTests, aboveMinIsValid)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(DurationSquaredTests, belowMinIsInvalid)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::DurationSquared::getMax().isValid());
}

TEST(DurationSquaredTests, aboveMaxIsInvalid)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, belowMaxIsValid)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_DURATIONSQUARED_THROWS_EXCEPTION == 1)
TEST(DurationSquaredTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::DurationSquared value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(DurationSquaredTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::DurationSquared value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(DurationSquaredTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::DurationSquared value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(DurationSquaredTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DurationSquared::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DurationSquared>::lowest()));
}

TEST(DurationSquaredTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DurationSquared::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DurationSquared>::max()));
}

TEST(DurationSquaredTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::DurationSquared::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::DurationSquared>::epsilon()));
}

TEST(DurationSquaredTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::DurationSquared(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::DurationSquared(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::physics::DurationSquared::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::physics::DurationSquared(::ad::physics::DurationSquared::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DurationSquared::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DurationSquared::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::DurationSquared(-::ad::physics::DurationSquared::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DurationSquared::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::DurationSquared::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::DurationSquared(-::ad::physics::DurationSquared::cMaxValue))));
}

TEST(DurationSquaredTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::DurationSquared::cMinValue;
  ::ad::physics::DurationSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(DurationSquaredTests, copyConstructionFromValidValue)
{
  ::ad::physics::DurationSquared const validValue(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(DurationSquaredTests, moveConstructionFromValidValue)
{
  ::ad::physics::DurationSquared validValue(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::DurationSquared::cMinValue, static_cast<double>(value));
}

TEST(DurationSquaredTests, assignmentFromValidValue)
{
  ::ad::physics::DurationSquared const validValue(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(DurationSquaredTests, moveAssignmentFromValidValue)
{
  ::ad::physics::DurationSquared validValue(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::DurationSquared::cMinValue, static_cast<double>(value));
}

TEST(DurationSquaredTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::DurationSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::DurationSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::DurationSquared value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, assignmentFromInvalidValue)
{
  ::ad::physics::DurationSquared const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::DurationSquared value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(DurationSquaredTests, selfAssignment)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(DurationSquaredTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::DurationSquared value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_DURATIONSQUARED_THROWS_EXCEPTION == 1)
TEST(DurationSquaredTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::DurationSquared const value(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared const invalidValue;

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

TEST(DurationSquaredTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::DurationSquared const minimalValue(::ad::physics::DurationSquared::cMinValue);
  ::ad::physics::DurationSquared const maximalValue(::ad::physics::DurationSquared::cMaxValue);
  ::ad::physics::DurationSquared const invalidValue;
  ::ad::physics::DurationSquared calculationValue;

  //  operator+(::ad::physics::DurationSquared)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::DurationSquared)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::DurationSquared)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::DurationSquared)
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

  //  operator/(::ad::physics::DurationSquared)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::DurationSquared(0.0), std::out_of_range);

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

TEST(DurationSquaredTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::DurationSquared::cPrecisionValue * 10.;
  ::ad::physics::DurationSquared value;
  if (::ad::physics::DurationSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::DurationSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::DurationSquared(precisionValueTimesTen);
  }
  ::ad::physics::DurationSquared const sameValue = value;
  ::ad::physics::DurationSquared const slightlyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::DurationSquared::cPrecisionValue * 0.9);
  ::ad::physics::DurationSquared const slightlySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::DurationSquared::cPrecisionValue * 0.9);
  ::ad::physics::DurationSquared const actuallyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::DurationSquared::cPrecisionValue * 1.1);
  ::ad::physics::DurationSquared const actuallySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::DurationSquared::cPrecisionValue * 1.1);

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

TEST(DurationSquaredTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::DurationSquared::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::DurationSquared::cPrecisionValue * 10.;
  ::ad::physics::DurationSquared value;
  if (::ad::physics::DurationSquared::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::DurationSquared::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::DurationSquared(::ad::physics::DurationSquared::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::DurationSquared(precisionValueTimesTen);
  }

  ::ad::physics::DurationSquared result;

  //  operator+(::ad::physics::DurationSquared)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::DurationSquared)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::DurationSquared)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::DurationSquared)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  std::sqrt()
  ::ad::physics::Duration const squareRootResult = std::sqrt(value);
  EXPECT_NEAR(std::sqrt(static_cast<double>(value)), static_cast<double>(squareRootResult), cDoubleNear);

  //  operator*(double, ::ad::physics::DurationSquared)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::DurationSquared)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::DurationSquared::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::DurationSquared::cMaxValue))
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
