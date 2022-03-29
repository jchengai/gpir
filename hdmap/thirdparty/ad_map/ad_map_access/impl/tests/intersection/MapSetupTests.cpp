// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <gtest/gtest.h>
#include "MapSetup.hpp"

TEST(MapSetupTests, check_map_setups)
{
  using namespace ::map_setup;
  ASSERT_NO_THROW(prepareMapAllWayStop());
  ASSERT_NO_THROW(prepareMapBasicYield());
  ASSERT_NO_THROW(prepareMapBasicPriorityToRight());
  ASSERT_NO_THROW(prepareMapTpkPfzDrive());
  ASSERT_NO_THROW(prepareMapTrafficLightsPfz());
  ASSERT_NO_THROW(prepareMapSolidTrafficLights());
}
