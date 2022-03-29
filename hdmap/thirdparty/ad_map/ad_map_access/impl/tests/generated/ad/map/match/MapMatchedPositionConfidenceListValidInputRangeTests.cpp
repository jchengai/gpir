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

#include "ad/map/match/MapMatchedPositionConfidenceListValidInputRange.hpp"

TEST(MapMatchedPositionConfidenceListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::match::MapMatchedPositionConfidenceList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MapMatchedPositionConfidenceListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::match::MapMatchedPositionConfidenceList value;
  ::ad::map::match::MapMatchedPosition element;
  ::ad::map::match::LanePoint elementLanePoint;
  ::ad::map::point::ParaPoint elementLanePointParaPoint;
  ::ad::map::lane::LaneId elementLanePointParaPointLaneId(1);
  elementLanePointParaPoint.laneId = elementLanePointParaPointLaneId;
  ::ad::physics::ParametricValue elementLanePointParaPointParametricOffset(0.);
  elementLanePointParaPoint.parametricOffset = elementLanePointParaPointParametricOffset;
  elementLanePoint.paraPoint = elementLanePointParaPoint;
  ::ad::physics::RatioValue elementLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  elementLanePoint.lateralT = elementLanePointLateralT;
  ::ad::physics::Distance elementLanePointLaneLength(-1e9);
  elementLanePoint.laneLength = elementLanePointLaneLength;
  ::ad::physics::Distance elementLanePointLaneWidth(-1e9);
  elementLanePoint.laneWidth = elementLanePointLaneWidth;
  element.lanePoint = elementLanePoint;
  ::ad::map::match::MapMatchedPositionType elementType(::ad::map::match::MapMatchedPositionType::INVALID);
  element.type = elementType;
  ::ad::map::point::ECEFPoint elementMatchedPoint;
  ::ad::map::point::ECEFCoordinate elementMatchedPointX(-6400000);
  elementMatchedPoint.x = elementMatchedPointX;
  ::ad::map::point::ECEFCoordinate elementMatchedPointY(-6400000);
  elementMatchedPoint.y = elementMatchedPointY;
  ::ad::map::point::ECEFCoordinate elementMatchedPointZ(-6400000);
  elementMatchedPoint.z = elementMatchedPointZ;
  element.matchedPoint = elementMatchedPoint;
  ::ad::physics::Probability elementProbability(0.);
  element.probability = elementProbability;
  ::ad::map::point::ECEFPoint elementQueryPoint;
  ::ad::map::point::ECEFCoordinate elementQueryPointX(-6400000);
  elementQueryPoint.x = elementQueryPointX;
  ::ad::map::point::ECEFCoordinate elementQueryPointY(-6400000);
  elementQueryPoint.y = elementQueryPointY;
  ::ad::map::point::ECEFCoordinate elementQueryPointZ(-6400000);
  elementQueryPoint.z = elementQueryPointZ;
  element.queryPoint = elementQueryPoint;
  ::ad::physics::Distance elementMatchedPointDistance(-1e9);
  element.matchedPointDistance = elementMatchedPointDistance;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MapMatchedPositionConfidenceListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::match::MapMatchedPositionConfidenceList value;
  ::ad::map::match::MapMatchedPosition element;
  ::ad::map::match::LanePoint elementLanePoint;
  ::ad::map::point::ParaPoint elementLanePointParaPoint;
  ::ad::physics::ParametricValue elementLanePointParaPointParametricOffset(
    0. - ::ad::physics::ParametricValue::cPrecisionValue);
  elementLanePointParaPoint.parametricOffset = elementLanePointParaPointParametricOffset;
  elementLanePoint.paraPoint = elementLanePointParaPoint;
  element.lanePoint = elementLanePoint;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
