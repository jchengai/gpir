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
#include "ad/physics/Weight.hpp"

TEST(WeightTests, defaultConstructionIsInvalid)
{
  ::ad::physics::Weight value;
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::physics::Weight::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-3, ::ad::physics::Weight::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::physics::Weight::cPrecisionValue, static_cast<double>(::ad::physics::Weight::getPrecision()));
}

TEST(WeightTests, minIsValid)
{
  EXPECT_TRUE(::ad::physics::Weight::getMin().isValid());
}

TEST(WeightTests, aboveMinIsValid)
{
  ::ad::physics::Weight value(::ad::physics::Weight::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(WeightTests, belowMinIsInvalid)
{
  ::ad::physics::Weight value(::ad::physics::Weight::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, maxIsValid)
{
  EXPECT_TRUE(::ad::physics::Weight::getMax().isValid());
}

TEST(WeightTests, aboveMaxIsInvalid)
{
  ::ad::physics::Weight value(::ad::physics::Weight::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, belowMaxIsValid)
{
  ::ad::physics::Weight value(::ad::physics::Weight::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_PHYSICS_WEIGHT_THROWS_EXCEPTION == 1)
TEST(WeightTests, ensureValidThrowsOnInvalid)
{
  ::ad::physics::Weight value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(WeightTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::physics::Weight value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(WeightTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::physics::Weight value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(WeightTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::Weight::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::physics::Weight>::lowest()));
}

TEST(WeightTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::Weight::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::physics::Weight>::max()));
}

TEST(WeightTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::physics::Weight::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::physics::Weight>::epsilon()));
}

TEST(WeightTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::physics::Weight(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::physics::Weight(-1.))));
  EXPECT_DOUBLE_EQ(::ad::physics::Weight::cPrecisionValue,
                   static_cast<double>(std::fabs(::ad::physics::Weight(::ad::physics::Weight::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::Weight::cMinValue),
                   static_cast<double>(std::fabs(::ad::physics::Weight(::ad::physics::Weight::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::Weight::cMinValue),
                   static_cast<double>(std::fabs(::ad::physics::Weight(-::ad::physics::Weight::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::Weight::cMaxValue),
                   static_cast<double>(std::fabs(::ad::physics::Weight(::ad::physics::Weight::cMaxValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::physics::Weight::cMaxValue),
                   static_cast<double>(std::fabs(::ad::physics::Weight(-::ad::physics::Weight::cMaxValue))));
}

TEST(WeightTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::physics::Weight::cMinValue;
  ::ad::physics::Weight value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(WeightTests, copyConstructionFromValidValue)
{
  ::ad::physics::Weight const validValue(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(WeightTests, moveConstructionFromValidValue)
{
  ::ad::physics::Weight validValue(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::Weight::cMinValue, static_cast<double>(value));
}

TEST(WeightTests, assignmentFromValidValue)
{
  ::ad::physics::Weight const validValue(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(WeightTests, moveAssignmentFromValidValue)
{
  ::ad::physics::Weight validValue(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::physics::Weight::cMinValue, static_cast<double>(value));
}

TEST(WeightTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::physics::Weight value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, copyConstructionFromInvalidValue)
{
  ::ad::physics::Weight const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::Weight value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, assignmentFromInvalidValue)
{
  ::ad::physics::Weight const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::physics::Weight value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(WeightTests, selfAssignment)
{
  ::ad::physics::Weight value(::ad::physics::Weight::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(WeightTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::physics::Weight value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_PHYSICS_WEIGHT_THROWS_EXCEPTION == 1)
TEST(WeightTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::physics::Weight const value(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight const invalidValue;

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

TEST(WeightTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::physics::Weight const minimalValue(::ad::physics::Weight::cMinValue);
  ::ad::physics::Weight const maximalValue(::ad::physics::Weight::cMaxValue);
  ::ad::physics::Weight const invalidValue;
  ::ad::physics::Weight calculationValue;

  //  operator+(::ad::physics::Weight)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::physics::Weight)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::physics::Weight)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::physics::Weight)
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

  //  operator/(::ad::physics::Weight)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::physics::Weight(0.0), std::out_of_range);

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

TEST(WeightTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::physics::Weight::cPrecisionValue * 10.;
  ::ad::physics::Weight value;
  if (::ad::physics::Weight::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::Weight(::ad::physics::Weight::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::Weight::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::Weight(::ad::physics::Weight::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::Weight(precisionValueTimesTen);
  }
  ::ad::physics::Weight const sameValue = value;
  ::ad::physics::Weight const slightlyBiggerValue(static_cast<double>(value)
                                                  + ::ad::physics::Weight::cPrecisionValue * 0.9);
  ::ad::physics::Weight const slightlySmallerValue(static_cast<double>(value)
                                                   - ::ad::physics::Weight::cPrecisionValue * 0.9);
  ::ad::physics::Weight const actuallyBiggerValue(static_cast<double>(value)
                                                  + ::ad::physics::Weight::cPrecisionValue * 1.1);
  ::ad::physics::Weight const actuallySmallerValue(static_cast<double>(value)
                                                   - ::ad::physics::Weight::cPrecisionValue * 1.1);

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

TEST(WeightTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::physics::Weight::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::physics::Weight::cPrecisionValue * 10.;
  ::ad::physics::Weight value;
  if (::ad::physics::Weight::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::physics::Weight(::ad::physics::Weight::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::physics::Weight::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::physics::Weight(::ad::physics::Weight::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::physics::Weight(precisionValueTimesTen);
  }

  ::ad::physics::Weight result;

  //  operator+(::ad::physics::Weight)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::physics::Weight)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::physics::Weight)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::physics::Weight)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::physics::Weight)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::physics::Weight)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::physics::Weight::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::physics::Weight::cMaxValue))
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
