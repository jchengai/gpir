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
#include "ad/map/point/ECEFCoordinate.hpp"

TEST(ECEFCoordinateTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::ECEFCoordinate value;
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, minIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(-1e9, ::ad::map::point::ECEFCoordinate::cMinValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cMinValue,
                   static_cast<double>(::ad::map::point::ECEFCoordinate::getMin()));
}

TEST(ECEFCoordinateTests, maxIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(1e9, ::ad::map::point::ECEFCoordinate::cMaxValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cMaxValue,
                   static_cast<double>(::ad::map::point::ECEFCoordinate::getMax()));
}

TEST(ECEFCoordinateTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::ECEFCoordinate::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-3, ::ad::map::point::ECEFCoordinate::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cPrecisionValue,
                   static_cast<double>(::ad::map::point::ECEFCoordinate::getPrecision()));
}

TEST(ECEFCoordinateTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::ECEFCoordinate::getMin().isValid());
}

TEST(ECEFCoordinateTests, aboveMinIsValid)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(ECEFCoordinateTests, belowMinIsInvalid)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::ECEFCoordinate::getMax().isValid());
}

TEST(ECEFCoordinateTests, aboveMaxIsInvalid)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, belowMaxIsValid)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_ECEFCOORDINATE_THROWS_EXCEPTION == 1)
TEST(ECEFCoordinateTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::ECEFCoordinate value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(ECEFCoordinateTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::ECEFCoordinate value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(ECEFCoordinateTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::ECEFCoordinate value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(ECEFCoordinateTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ECEFCoordinate::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ECEFCoordinate>::lowest()));
}

TEST(ECEFCoordinateTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ECEFCoordinate::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ECEFCoordinate>::max()));
}

TEST(ECEFCoordinateTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ECEFCoordinate::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ECEFCoordinate>::epsilon()));
}

TEST(ECEFCoordinateTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(-1.))));
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cPrecisionValue,
                   static_cast<double>(
                     std::fabs(::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ECEFCoordinate::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ECEFCoordinate::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(-::ad::map::point::ECEFCoordinate::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ECEFCoordinate::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ECEFCoordinate::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ECEFCoordinate(-::ad::map::point::ECEFCoordinate::cMaxValue))));
}

TEST(ECEFCoordinateTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::ECEFCoordinate::cMinValue;
  ::ad::map::point::ECEFCoordinate value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(ECEFCoordinateTests, copyConstructionFromValidValue)
{
  ::ad::map::point::ECEFCoordinate const validValue(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ECEFCoordinateTests, moveConstructionFromValidValue)
{
  ::ad::map::point::ECEFCoordinate validValue(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cMinValue, static_cast<double>(value));
}

TEST(ECEFCoordinateTests, assignmentFromValidValue)
{
  ::ad::map::point::ECEFCoordinate const validValue(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ECEFCoordinateTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::ECEFCoordinate validValue(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ECEFCoordinate::cMinValue, static_cast<double>(value));
}

TEST(ECEFCoordinateTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::ECEFCoordinate value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::ECEFCoordinate const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ECEFCoordinate value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, assignmentFromInvalidValue)
{
  ::ad::map::point::ECEFCoordinate const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ECEFCoordinate value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(ECEFCoordinateTests, selfAssignment)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(ECEFCoordinateTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::ECEFCoordinate value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_ECEFCOORDINATE_THROWS_EXCEPTION == 1)
TEST(ECEFCoordinateTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::ECEFCoordinate const value(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate const invalidValue;

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

TEST(ECEFCoordinateTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::ECEFCoordinate const minimalValue(::ad::map::point::ECEFCoordinate::cMinValue);
  ::ad::map::point::ECEFCoordinate const maximalValue(::ad::map::point::ECEFCoordinate::cMaxValue);
  ::ad::map::point::ECEFCoordinate const invalidValue;
  ::ad::map::point::ECEFCoordinate calculationValue;

  //  operator+(::ad::map::point::ECEFCoordinate)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::ECEFCoordinate)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::ECEFCoordinate)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::ECEFCoordinate)
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

  //  operator/(::ad::map::point::ECEFCoordinate)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::ECEFCoordinate(0.0), std::out_of_range);

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

TEST(ECEFCoordinateTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::ECEFCoordinate::cPrecisionValue * 10.;
  ::ad::map::point::ECEFCoordinate value;
  if (::ad::map::point::ECEFCoordinate::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ECEFCoordinate::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ECEFCoordinate(precisionValueTimesTen);
  }
  ::ad::map::point::ECEFCoordinate const sameValue = value;
  ::ad::map::point::ECEFCoordinate const slightlyBiggerValue(static_cast<double>(value)
                                                             + ::ad::map::point::ECEFCoordinate::cPrecisionValue * 0.9);
  ::ad::map::point::ECEFCoordinate const slightlySmallerValue(
    static_cast<double>(value) - ::ad::map::point::ECEFCoordinate::cPrecisionValue * 0.9);
  ::ad::map::point::ECEFCoordinate const actuallyBiggerValue(static_cast<double>(value)
                                                             + ::ad::map::point::ECEFCoordinate::cPrecisionValue * 1.1);
  ::ad::map::point::ECEFCoordinate const actuallySmallerValue(
    static_cast<double>(value) - ::ad::map::point::ECEFCoordinate::cPrecisionValue * 1.1);

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

TEST(ECEFCoordinateTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::ECEFCoordinate::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::ECEFCoordinate::cPrecisionValue * 10.;
  ::ad::map::point::ECEFCoordinate value;
  if (::ad::map::point::ECEFCoordinate::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ECEFCoordinate::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ECEFCoordinate(::ad::map::point::ECEFCoordinate::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ECEFCoordinate(precisionValueTimesTen);
  }

  ::ad::map::point::ECEFCoordinate result;

  //  operator+(::ad::map::point::ECEFCoordinate)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::ECEFCoordinate)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::ECEFCoordinate)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::ECEFCoordinate)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::ECEFCoordinate)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::ECEFCoordinate)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::ECEFCoordinate::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::ECEFCoordinate::cMaxValue))
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
