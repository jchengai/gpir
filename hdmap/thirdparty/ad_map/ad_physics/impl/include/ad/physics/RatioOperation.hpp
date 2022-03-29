// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include "ad/physics/Types.hpp"

/* ##################  ad::physics::Acceleration  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Acceleration operator*(ad::physics::Acceleration const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Acceleration const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Acceleration operator*(ad::physics::RatioValue const &r, ad::physics::Acceleration const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Acceleration operator/(ad::physics::Acceleration const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Acceleration const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Angle  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Angle operator*(ad::physics::Angle const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Angle const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Angle operator*(ad::physics::RatioValue const &r, ad::physics::Angle const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Angle operator/(ad::physics::Angle const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Angle const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::AngularAcceleration  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::AngularAcceleration operator*(ad::physics::AngularAcceleration const &value,
                                                  ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::AngularAcceleration const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::AngularAcceleration operator*(ad::physics::RatioValue const &r,
                                                  ad::physics::AngularAcceleration const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::AngularAcceleration operator/(ad::physics::AngularAcceleration const &value,
                                                  ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::AngularAcceleration const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::AngularVelocity  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::AngularVelocity operator*(ad::physics::AngularVelocity const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::AngularVelocity const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::AngularVelocity operator*(ad::physics::RatioValue const &r,
                                              ad::physics::AngularVelocity const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::AngularVelocity operator/(ad::physics::AngularVelocity const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::AngularVelocity const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Distance  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Distance operator*(ad::physics::Distance const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Distance const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Distance operator*(ad::physics::RatioValue const &r, ad::physics::Distance const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Distance operator/(ad::physics::Distance const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Distance const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::DistanceSquared  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::DistanceSquared operator*(ad::physics::DistanceSquared const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::DistanceSquared const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::DistanceSquared operator*(ad::physics::RatioValue const &r,
                                              ad::physics::DistanceSquared const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::DistanceSquared operator/(ad::physics::DistanceSquared const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::DistanceSquared const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Duration  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Duration operator*(ad::physics::Duration const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Duration const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Duration operator*(ad::physics::RatioValue const &r, ad::physics::Duration const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Duration operator/(ad::physics::Duration const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Duration const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::DurationSquared  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::DurationSquared operator*(ad::physics::DurationSquared const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::DurationSquared const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::DurationSquared operator*(ad::physics::RatioValue const &r,
                                              ad::physics::DurationSquared const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::DurationSquared operator/(ad::physics::DurationSquared const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::DurationSquared const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::ParametricValue ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::ParametricValue operator*(ad::physics::ParametricValue const &value,
                                              ad::physics::RatioValue const &r)
{
  value.ensureValid();
  r.ensureValid();
  ad::physics::ParametricValue const result(static_cast<double>(value) * static_cast<double>(r));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::ParametricValue operator/(ad::physics::ParametricValue const &value,
                                              ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::ParametricValue const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Probability  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Probability operator*(ad::physics::Probability const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Probability const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Probability operator*(ad::physics::RatioValue const &r, ad::physics::Probability const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Probability operator/(ad::physics::Probability const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Probability const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::RatioValue  ################ */

/*!
 * @brief Arithmetic physics operation ratio = ratio * ratio
 *
 * @param[in] r1 ratio value
 * @param[in] r2 ratio value
 *
 * @returns ratio = r1 * r2
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::RatioValue operator*(ad::physics::RatioValue const &r1, ad::physics::RatioValue const &r2)
{
  r1.ensureValid();
  r2.ensureValid();
  ad::physics::RatioValue const result(static_cast<double>(r1) * static_cast<double>(r2));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Speed  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Speed operator*(ad::physics::Speed const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Speed const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Speed operator*(ad::physics::RatioValue const &r, ad::physics::Speed const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Speed operator/(ad::physics::Speed const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Speed const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::SpeedSquared  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::SpeedSquared operator*(ad::physics::SpeedSquared const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::SpeedSquared const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::SpeedSquared operator*(ad::physics::RatioValue const &r, ad::physics::SpeedSquared const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::SpeedSquared operator/(ad::physics::SpeedSquared const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::SpeedSquared const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}

/* ##################  ad::physics::Weight  ################ */

/*!
 * @brief Arithmetic physics operation value = value * r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value * r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Weight operator*(ad::physics::Weight const &value, ad::physics::RatioValue const &r)
{
  r.ensureValid();
  value.ensureValid();
  ad::physics::Weight const result(static_cast<double>(r) * static_cast<double>(value));
  result.ensureValid();
  return result;
}

/*!
 * @brief Arithmetic ratio operation value = r * value
 *
 * @param[in] r ratio value
 * @param[in] value physics value
 *
 * @returns value = r * value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Weight operator*(ad::physics::RatioValue const &r, ad::physics::Weight const &value)
{
  return operator*(value, r);
}

/*!
 * @brief Arithmetic physics operation value = value / r
 *
 * @param[in] value physics value
 * @param[in] r ratio value
 *
 * @returns value = value / r
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Weight operator/(ad::physics::Weight const &value, ad::physics::RatioValue const &r)
{
  r.ensureValidNonZero();
  value.ensureValid();
  ad::physics::Weight const result(static_cast<double>(value) / static_cast<double>(r));
  result.ensureValid();
  return result;
}
