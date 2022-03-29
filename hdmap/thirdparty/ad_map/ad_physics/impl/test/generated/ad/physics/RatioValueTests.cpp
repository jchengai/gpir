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
#include "ad/physics/RatioValue.hpp"

TEST(RatioValueTests, defaultConstructionIsInvalid)
{
  ::ad::physics::RatioValue value;
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::RatioValue::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-9, ::ad::physics::RatioValue::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::RatioValue::cPrecisionValue,
                   static_cast<double>(::ad::physics::RatioValue::getPrecision()));
}

TEST(RatioValueTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::RatioValue::getMin().isValid());
}

TEST(RatioValueTests, aboveMinIsValid)
{
  ::ad::physics::RatioValue value(::ad::physics::RatioValue::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(RatioValueTests, belowMinIsInvalid)
{
  ::ad::physics::RatioValue value(::ad::physics::RatioValue::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::RatioValue::getMax().isValid());
}

TEST(RatioValueTests, aboveMaxIsInvalid)
{
  ::ad::physics::RatioValue value(::ad::physics::RatioValue::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, belowMaxIsValid)
{
  ::ad::physics::RatioValue value(::ad::physics::RatioValue::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_RATIOVALUE_THROWS_EXCEPTION == 1)
TEST(RatioValueTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::RatioValue value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(RatioValueTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::RatioValue value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(RatioValueTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::RatioValue value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(RatioValueTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::RatioValue::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::RatioValue>::lowest()));
}

TEST(RatioValueTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::RatioValue::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::RatioValue>::max()));
}

TEST(RatioValueTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::RatioValue::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::RatioValue>::epsilon()));
}

TEST(RatioValueTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::RatioValue(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::RatioValue(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::physics::RatioValue::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::physics::RatioValue(::ad::physics::RatioValue::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::RatioValue::cMinValue),
                   static_cast<double>(std::fabs(::ad::physics::RatioValue(::ad::physics::RatioValue::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::RatioValue::cMinValue),
                   static_cast<double>(std::fabs(::ad::physics::RatioValue(-::ad::physics::RatioValue::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::RatioValue::cMaxValue),
                   static_cast<double>(std::fabs(::ad::physics::RatioValue(::ad::physics::RatioValue::cMaxValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::RatioValue::cMaxValue),
                   static_cast<double>(std::fabs(::ad::physics::RatioValue(-::ad::physics::RatioValue::cMaxValue))));
}

TEST(RatioValueTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::RatioValue::cMinValue;
  ::ad::physics::RatioValue value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(RatioValueTests, copyConstructionFromValidValue)
{
  ::ad::physics::RatioValue const validValue(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(RatioValueTests, moveConstructionFromValidValue)
{
  ::ad::physics::RatioValue validValue(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::RatioValue::cMinValue, static_cast<double>(value));
}

TEST(RatioValueTests, assignmentFromValidValue)
{
  ::ad::physics::RatioValue const validValue(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(RatioValueTests, moveAssignmentFromValidValue)
{
  ::ad::physics::RatioValue validValue(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::RatioValue::cMinValue, static_cast<double>(value));
}

TEST(RatioValueTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::RatioValue value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::RatioValue const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::RatioValue value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, assignmentFromInvalidValue)
{
  ::ad::physics::RatioValue const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::RatioValue value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(RatioValueTests, selfAssignment)
{
  ::ad::physics::RatioValue value(::ad::physics::RatioValue::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(RatioValueTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::RatioValue value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_RATIOVALUE_THROWS_EXCEPTION == 1)
TEST(RatioValueTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::RatioValue const value(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue const invalidValue;

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

TEST(RatioValueTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::RatioValue const minimalValue(::ad::physics::RatioValue::cMinValue);
  ::ad::physics::RatioValue const maximalValue(::ad::physics::RatioValue::cMaxValue);
  ::ad::physics::RatioValue const invalidValue;
  ::ad::physics::RatioValue calculationValue;

  //  operator+(::ad::physics::RatioValue)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::RatioValue)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::RatioValue)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::RatioValue)
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

  //  operator/(::ad::physics::RatioValue)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::RatioValue(0.0), std::out_of_range);

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

TEST(RatioValueTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::RatioValue::cPrecisionValue * 10.;
  ::ad::physics::RatioValue value;
  if (::ad::physics::RatioValue::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::RatioValue(::ad::physics::RatioValue::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::RatioValue::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::RatioValue(::ad::physics::RatioValue::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::RatioValue(precisionValueTimesTen);
  }
  ::ad::physics::RatioValue const sameValue = value;
  ::ad::physics::RatioValue const slightlyBiggerValue(static_cast<double>(value)
                                                      + ::ad::physics::RatioValue::cPrecisionValue * 0.9);
  ::ad::physics::RatioValue const slightlySmallerValue(static_cast<double>(value)
                                                       - ::ad::physics::RatioValue::cPrecisionValue * 0.9);
  ::ad::physics::RatioValue const actuallyBiggerValue(static_cast<double>(value)
                                                      + ::ad::physics::RatioValue::cPrecisionValue * 1.1);
  ::ad::physics::RatioValue const actuallySmallerValue(static_cast<double>(value)
                                                       - ::ad::physics::RatioValue::cPrecisionValue * 1.1);

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

TEST(RatioValueTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::RatioValue::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::RatioValue::cPrecisionValue * 10.;
  ::ad::physics::RatioValue value;
  if (::ad::physics::RatioValue::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::RatioValue(::ad::physics::RatioValue::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::RatioValue::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::RatioValue(::ad::physics::RatioValue::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::RatioValue(precisionValueTimesTen);
  }

  ::ad::physics::RatioValue result;

  //  operator+(::ad::physics::RatioValue)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::RatioValue)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::RatioValue)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::RatioValue)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::physics::RatioValue)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::RatioValue)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::RatioValue::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::RatioValue::cMaxValue))
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
