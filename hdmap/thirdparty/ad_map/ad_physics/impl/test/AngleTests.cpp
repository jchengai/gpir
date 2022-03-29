// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "gtest/gtest.h"
#include "ad/physics/AngleOperation.hpp"

TEST(AngleTests, normalizeAngle)
{
  ASSERT_EQ(::ad::physics::cPI, ::ad::physics::normalizeAngle(ad::physics::Angle(M_PI)));
  ASSERT_EQ(::ad::physics::Angle(0.0), ::ad::physics::normalizeAngle(ad::physics::Angle(2 * M_PI)));
  ASSERT_EQ(::ad::physics::cPI, ::ad::physics::normalizeAngle(ad::physics::Angle(-5 * M_PI)));
  ASSERT_EQ(::ad::physics::Angle(0.0), ::ad::physics::normalizeAngle(ad::physics::Angle(6 * M_PI)));
}

TEST(AngleTests, normalizeAngleSigned)
{
  ASSERT_EQ(::ad::physics::Angle(M_PI), ::ad::physics::normalizeAngleSigned(ad::physics::Angle(M_PI)));
  ASSERT_EQ(::ad::physics::Angle(0.0), ::ad::physics::normalizeAngleSigned(ad::physics::Angle(2 * M_PI)));
  ASSERT_EQ(::ad::physics::Angle(-M_PI + 0.001),
            ::ad::physics::normalizeAngleSigned(ad::physics::Angle(-4.9999 * M_PI)));
  ASSERT_EQ(::ad::physics::Angle(M_PI), ::ad::physics::normalizeAngleSigned(ad::physics::Angle(-5 * M_PI)));
  ASSERT_EQ(::ad::physics::Angle(0.0), ::ad::physics::normalizeAngleSigned(ad::physics::Angle(6 * M_PI)));
}
