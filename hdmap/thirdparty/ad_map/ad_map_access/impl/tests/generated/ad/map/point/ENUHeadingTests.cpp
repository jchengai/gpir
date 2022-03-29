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
#include "ad/map/point/ENUHeading.hpp"

TEST(ENUHeadingTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::ENUHeading value;
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::ENUHeading::cPrecisionValue);
  EXPECT_DOUBLE_EQ(0.0001, ::ad::map::point::ENUHeading::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUHeading::cPrecisionValue,
                   static_cast<double>(::ad::map::point::ENUHeading::getPrecision()));
}

TEST(ENUHeadingTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::ENUHeading::getMin().isValid());
}

TEST(ENUHeadingTests, aboveMinIsValid)
{
  ::ad::map::point::ENUHeading value(::ad::map::point::ENUHeading::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(ENUHeadingTests, belowMinIsInvalid)
{
  ::ad::map::point::ENUHeading value(::ad::map::point::ENUHeading::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::ENUHeading::getMax().isValid());
}

TEST(ENUHeadingTests, aboveMaxIsInvalid)
{
  ::ad::map::point::ENUHeading value(::ad::map::point::ENUHeading::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, belowMaxIsValid)
{
  ::ad::map::point::ENUHeading value(::ad::map::point::ENUHeading::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_ENUHEADING_THROWS_EXCEPTION == 1)
TEST(ENUHeadingTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::ENUHeading value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(ENUHeadingTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::ENUHeading value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(ENUHeadingTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::ENUHeading value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(ENUHeadingTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUHeading::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUHeading>::lowest()));
}

TEST(ENUHeadingTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUHeading::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUHeading>::max()));
}

TEST(ENUHeadingTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUHeading::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUHeading>::epsilon()));
}

TEST(ENUHeadingTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::ENUHeading(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::ENUHeading(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::map::point::ENUHeading::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUHeading::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUHeading::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUHeading(-::ad::map::point::ENUHeading::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUHeading::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUHeading::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUHeading(-::ad::map::point::ENUHeading::cMaxValue))));
}

TEST(ENUHeadingTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::ENUHeading::cMinValue;
  ::ad::map::point::ENUHeading value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(ENUHeadingTests, copyConstructionFromValidValue)
{
  ::ad::map::point::ENUHeading const validValue(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ENUHeadingTests, moveConstructionFromValidValue)
{
  ::ad::map::point::ENUHeading validValue(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUHeading::cMinValue, static_cast<double>(value));
}

TEST(ENUHeadingTests, assignmentFromValidValue)
{
  ::ad::map::point::ENUHeading const validValue(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ENUHeadingTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::ENUHeading validValue(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUHeading::cMinValue, static_cast<double>(value));
}

TEST(ENUHeadingTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::ENUHeading value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::ENUHeading const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ENUHeading value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, assignmentFromInvalidValue)
{
  ::ad::map::point::ENUHeading const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ENUHeading value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(ENUHeadingTests, selfAssignment)
{
  ::ad::map::point::ENUHeading value(::ad::map::point::ENUHeading::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(ENUHeadingTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::ENUHeading value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_ENUHEADING_THROWS_EXCEPTION == 1)
TEST(ENUHeadingTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::ENUHeading const value(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading const invalidValue;

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

TEST(ENUHeadingTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::ENUHeading const minimalValue(::ad::map::point::ENUHeading::cMinValue);
  ::ad::map::point::ENUHeading const maximalValue(::ad::map::point::ENUHeading::cMaxValue);
  ::ad::map::point::ENUHeading const invalidValue;
  ::ad::map::point::ENUHeading calculationValue;

  //  operator+(::ad::map::point::ENUHeading)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::ENUHeading)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::ENUHeading)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::ENUHeading)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue -= minimalValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= invalidValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= maximalValue, std::out_of_range);

  //  operator*(double)
  EXPECT_THROW(invalidValue * static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue * static_cast<double>(maximalValue), std::out_of_range);

  //  operator/(double)
  EXPECT_THROW(invalidValue / static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue / static_cast<double>(invalidValue), std::out_of_range);
  EXPECT_THROW(maximalValue / 0.0, std::out_of_range);
  EXPECT_THROW(maximalValue / 0.5, std::out_of_range);

  //  operator/(::ad::map::point::ENUHeading)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::ENUHeading(0.0), std::out_of_range);

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

TEST(ENUHeadingTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::ENUHeading::cPrecisionValue * 10.;
  ::ad::map::point::ENUHeading value;
  if (::ad::map::point::ENUHeading::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ENUHeading::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ENUHeading(precisionValueTimesTen);
  }
  ::ad::map::point::ENUHeading const sameValue = value;
  ::ad::map::point::ENUHeading const slightlyBiggerValue(static_cast<double>(value)
                                                         + ::ad::map::point::ENUHeading::cPrecisionValue * 0.9);
  ::ad::map::point::ENUHeading const slightlySmallerValue(static_cast<double>(value)
                                                          - ::ad::map::point::ENUHeading::cPrecisionValue * 0.9);
  ::ad::map::point::ENUHeading const actuallyBiggerValue(static_cast<double>(value)
                                                         + ::ad::map::point::ENUHeading::cPrecisionValue * 1.1);
  ::ad::map::point::ENUHeading const actuallySmallerValue(static_cast<double>(value)
                                                          - ::ad::map::point::ENUHeading::cPrecisionValue * 1.1);

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

TEST(ENUHeadingTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::ENUHeading::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::ENUHeading::cPrecisionValue * 10.;
  ::ad::map::point::ENUHeading value;
  if (::ad::map::point::ENUHeading::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ENUHeading::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUHeading(::ad::map::point::ENUHeading::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ENUHeading(precisionValueTimesTen);
  }

  ::ad::map::point::ENUHeading result;

  //  operator+(::ad::map::point::ENUHeading)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::ENUHeading)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::ENUHeading)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::ENUHeading)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::ENUHeading)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::ENUHeading)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::ENUHeading::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::ENUHeading::cMaxValue))
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
