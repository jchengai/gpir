// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include "ad/physics/Types.hpp"

/*!
 * @brief Arithmetic physics operation v = a * t
 *
 * @param[in] a acceleration value
 * @param[in] t duration value
 *
 * @returns v = a * t as speed value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Speed operator*(ad::physics::Acceleration const &a, ad::physics::Duration const &t)
{
  a.ensureValid();
  t.ensureValid();
  ad::physics::Speed const v(static_cast<double>(a) * static_cast<double>(t));
  v.ensureValid();
  return v;
}

/*!
 * @brief Arithmetic physics operation s = a * t^2
 *
 * @param[in] a acceleration value
 * @param[in] t2 duration squared value
 *
 * @returns s = a * t2 as distance value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Distance operator*(ad::physics::Acceleration const &a, ad::physics::DurationSquared const &t2)
{
  a.ensureValid();
  t2.ensureValid();
  ad::physics::Distance const s(static_cast<double>(a) * static_cast<double>(t2));
  s.ensureValid();
  return s;
}

/*!
 * @brief Arithmetic physics operation v^2 = a * s
 *
 * @param[in] a acceleration value
 * @param[in] s distance value
 *
 * @returns v2 = a * s as speed squared value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::SpeedSquared operator*(ad::physics::Acceleration const &a, ad::physics::Distance const &s)
{
  a.ensureValid();
  s.ensureValid();
  ad::physics::SpeedSquared const v2(static_cast<double>(a) * static_cast<double>(s));
  v2.ensureValid();
  return v2;
}

/*!
 * @brief Arithmetic physics operation v = t * a
 *
 * @param[in] t duration value
 * @param[in] a acceleration value
 *
 * @returns v = a * t as speed value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Speed operator*(ad::physics::Duration const &t, ad::physics::Acceleration const &a)
{
  return operator*(a, t);
}

/*!
 * @brief Arithmetic physics operation s = v * t
 *
 * @param[in] v speed value
 * @param[in] t duration value
 *
 * @returns s = v * t as distance value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Distance operator*(ad::physics::Speed const &v, ad::physics::Duration const &t)
{
  v.ensureValid();
  t.ensureValid();
  ad::physics::Distance const s(static_cast<double>(v) * static_cast<double>(t));
  s.ensureValid();
  return s;
}

/*!
 * @brief Arithmetic physics operation s = t * v
 *
 * @param[in] t duration value
 * @param[in] v speed value
 *
 * @returns s = v * t as distance value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid
 */
inline ad::physics::Distance operator*(ad::physics::Duration const &t, ad::physics::Speed const &v)
{
  return operator*(v, t);
}

/*!
 * @brief Arithmetic physics operation t = s / v
 *
 * @param[in] s distance value
 * @param[in] v speed value
 *
 * @returns t = s / v as duration value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Duration operator/(ad::physics::Distance const &s, ad::physics::Speed const &v)
{
  s.ensureValid();
  v.ensureValidNonZero();
  ad::physics::Duration const t(static_cast<double>(s) / static_cast<double>(v));
  t.ensureValid();
  return t;
}

/*!
 * @brief Arithmetic physics operation t = v / a
 *
 * @param[in] v speed value
 * @param[in] a acceleration value
 *
 * @returns t = v / a as duration value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Duration operator/(ad::physics::Speed const &v, ad::physics::Acceleration const &a)
{
  v.ensureValid();
  a.ensureValidNonZero();
  ad::physics::Duration const t(static_cast<double>(v) / static_cast<double>(a));
  t.ensureValid();
  return t;
}

/*!
 * @brief Arithmetic physics operation a = v / t
 *
 * @param[in] v speed value
 * @param[in] t duration value
 *
 * @returns a = v / t as acceleration value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Acceleration operator/(ad::physics::Speed const &v, ad::physics::Duration const &t)
{
  v.ensureValid();
  t.ensureValidNonZero();
  ad::physics::Acceleration const a(static_cast<double>(v) / static_cast<double>(t));
  a.ensureValid();
  return a;
}

/*!
 * @brief Arithmetic physics operation s = v^2 / a
 *
 * @param[in] v2 squared speed value
 * @param[in] a acceleration value
 *
 * @returns s = v^2 / a as distance value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::Distance operator/(ad::physics::SpeedSquared const &v2, ad::physics::Acceleration const &a)
{
  v2.ensureValid();
  a.ensureValidNonZero();
  ad::physics::Distance const t(static_cast<double>(v2) / static_cast<double>(a));
  t.ensureValid();
  return t;
}

/*!
 * @brief Arithmetic physics operation t^2 = s / a
 *
 * @param[in] s distance value
 * @param[in] a acceleration value
 *
 * @returns t^2 = s / a as squared duration value
 *
 * \note throws a std::out_of_range exception if one of the two operands or the result of
 *   the operation is not valid or if the divisor is zero
 */
inline ad::physics::DurationSquared operator/(ad::physics::Distance const &s, ad::physics::Acceleration const &a)
{
  s.ensureValid();
  a.ensureValidNonZero();
  ad::physics::DurationSquared const t2(static_cast<double>(s) / static_cast<double>(a));
  t2.ensureValid();
  return t2;
}
