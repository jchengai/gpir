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

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/access/MapMetaDataValidInputRange.hpp"

TEST(MapMetaDataValidInputRangeTests, testValidInputRange)
{
  ::ad::map::access::MapMetaData value;
  ::ad::map::access::TrafficType valueTrafficType(::ad::map::access::TrafficType::INVALID);
  value.trafficType = valueTrafficType;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MapMetaDataValidInputRangeTests, testValidInputRangeTrafficTypeTooSmall)
{
  ::ad::map::access::MapMetaData value;
  ::ad::map::access::TrafficType valueTrafficType(::ad::map::access::TrafficType::INVALID);
  value.trafficType = valueTrafficType;

  // override member with data type value below input range minimum
  ::ad::map::access::TrafficType invalidInitializedMember(static_cast<::ad::map::access::TrafficType>(-1));
  value.trafficType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMetaDataValidInputRangeTests, testValidInputRangeTrafficTypeTooBig)
{
  ::ad::map::access::MapMetaData value;
  ::ad::map::access::TrafficType valueTrafficType(::ad::map::access::TrafficType::INVALID);
  value.trafficType = valueTrafficType;

  // override member with data type value above input range maximum
  ::ad::map::access::TrafficType invalidInitializedMember(static_cast<::ad::map::access::TrafficType>(-1));
  value.trafficType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
