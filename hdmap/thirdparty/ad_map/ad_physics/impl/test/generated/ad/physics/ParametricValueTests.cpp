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
#include "ad/physics/ParametricValue.hpp"

TEST(ParametricValueTests, defaultConstructionIsInvalid)
{
  ::ad::physics::ParametricValue value;
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::ParametricValue::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-6, ::ad::physics::ParametricValue::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::ParametricValue::cPrecisionValue,
                   static_cast<double>(::ad::physics::ParametricValue::getPrecision()));
}

TEST(ParametricValueTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::ParametricValue::getMin().isValid());
}

TEST(ParametricValueTests, aboveMinIsValid)
{
  ::ad::physics::ParametricValue value(::ad::physics::ParametricValue::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(ParametricValueTests, belowMinIsInvalid)
{
  ::ad::physics::ParametricValue value(::ad::physics::ParametricValue::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::ParametricValue::getMax().isValid());
}

TEST(ParametricValueTests, aboveMaxIsInvalid)
{
  ::ad::physics::ParametricValue value(::ad::physics::ParametricValue::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, belowMaxIsValid)
{
  ::ad::physics::ParametricValue value(::ad::physics::ParametricValue::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_PARAMETRICVALUE_THROWS_EXCEPTION == 1)
TEST(ParametricValueTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::ParametricValue value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(ParametricValueTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::ParametricValue value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(ParametricValueTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::ParametricValue value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(ParametricValueTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::ParametricValue::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::ParametricValue>::lowest()));
}

TEST(ParametricValueTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::ParametricValue::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::ParametricValue>::max()));
}

TEST(ParametricValueTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::ParametricValue::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::ParametricValue>::epsilon()));
}

TEST(ParametricValueTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::ParametricValue(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::ParametricValue(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::physics::ParametricValue::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::physics::ParametricValue(::ad::physics::ParametricValue::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::ParametricValue::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::ParametricValue::cMinValue),
    static_cast<double>(std::fabs(::ad::physics::ParametricValue(-::ad::physics::ParametricValue::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::ParametricValue::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::physics::ParametricValue::cMaxValue),
    static_cast<double>(std::fabs(::ad::physics::ParametricValue(-::ad::physics::ParametricValue::cMaxValue))));
}

TEST(ParametricValueTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::ParametricValue::cMinValue;
  ::ad::physics::ParametricValue value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(ParametricValueTests, copyConstructionFromValidValue)
{
  ::ad::physics::ParametricValue const validValue(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ParametricValueTests, moveConstructionFromValidValue)
{
  ::ad::physics::ParametricValue validValue(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::ParametricValue::cMinValue, static_cast<double>(value));
}

TEST(ParametricValueTests, assignmentFromValidValue)
{
  ::ad::physics::ParametricValue const validValue(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ParametricValueTests, moveAssignmentFromValidValue)
{
  ::ad::physics::ParametricValue validValue(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::ParametricValue::cMinValue, static_cast<double>(value));
}

TEST(ParametricValueTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::ParametricValue value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::ParametricValue const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::ParametricValue value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, assignmentFromInvalidValue)
{
  ::ad::physics::ParametricValue const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::ParametricValue value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(ParametricValueTests, selfAssignment)
{
  ::ad::physics::ParametricValue value(::ad::physics::ParametricValue::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(ParametricValueTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::ParametricValue value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_PARAMETRICVALUE_THROWS_EXCEPTION == 1)
TEST(ParametricValueTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::ParametricValue const value(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue const invalidValue;

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

TEST(ParametricValueTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::ParametricValue const minimalValue(::ad::physics::ParametricValue::cMinValue);
  ::ad::physics::ParametricValue const maximalValue(::ad::physics::ParametricValue::cMaxValue);
  ::ad::physics::ParametricValue const invalidValue;
  ::ad::physics::ParametricValue calculationValue;

  //  operator+(::ad::physics::ParametricValue)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::ParametricValue)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::ParametricValue)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::ParametricValue)
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

  //  operator/(::ad::physics::ParametricValue)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::ParametricValue(0.0), std::out_of_range);

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

TEST(ParametricValueTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::ParametricValue::cPrecisionValue * 10.;
  ::ad::physics::ParametricValue value;
  if (::ad::physics::ParametricValue::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::ParametricValue::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::ParametricValue(precisionValueTimesTen);
  }
  ::ad::physics::ParametricValue const sameValue = value;
  ::ad::physics::ParametricValue const slightlyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::ParametricValue::cPrecisionValue * 0.9);
  ::ad::physics::ParametricValue const slightlySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::ParametricValue::cPrecisionValue * 0.9);
  ::ad::physics::ParametricValue const actuallyBiggerValue(static_cast<double>(value)
                                                           + ::ad::physics::ParametricValue::cPrecisionValue * 1.1);
  ::ad::physics::ParametricValue const actuallySmallerValue(static_cast<double>(value)
                                                            - ::ad::physics::ParametricValue::cPrecisionValue * 1.1);

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

TEST(ParametricValueTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::ParametricValue::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::ParametricValue::cPrecisionValue * 10.;
  ::ad::physics::ParametricValue value;
  if (::ad::physics::ParametricValue::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::ParametricValue::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::ParametricValue(::ad::physics::ParametricValue::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::ParametricValue(precisionValueTimesTen);
  }

  ::ad::physics::ParametricValue result;

  //  operator+(::ad::physics::ParametricValue)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::ParametricValue)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::ParametricValue)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::ParametricValue)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::physics::ParametricValue)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::ParametricValue)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::ParametricValue::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::ParametricValue::cMaxValue))
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
