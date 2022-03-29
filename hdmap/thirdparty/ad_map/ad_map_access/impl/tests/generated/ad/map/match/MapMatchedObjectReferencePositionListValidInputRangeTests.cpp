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

#include "ad/map/match/MapMatchedObjectReferencePositionListValidInputRange.hpp"

TEST(MapMatchedObjectReferencePositionListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::match::MapMatchedObjectReferencePositionList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MapMatchedObjectReferencePositionListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::match::MapMatchedObjectReferencePositionList value;
  ::ad::map::match::MapMatchedPositionConfidenceList element;
  ::ad::map::match::MapMatchedPosition elementElement;
  ::ad::map::match::LanePoint elementElementLanePoint;
  ::ad::map::point::ParaPoint elementElementLanePointParaPoint;
  ::ad::map::lane::LaneId elementElementLanePointParaPointLaneId(1);
  elementElementLanePointParaPoint.laneId = elementElementLanePointParaPointLaneId;
  ::ad::physics::ParametricValue elementElementLanePointParaPointParametricOffset(0.);
  elementElementLanePointParaPoint.parametricOffset = elementElementLanePointParaPointParametricOffset;
  elementElementLanePoint.paraPoint = elementElementLanePointParaPoint;
  ::ad::physics::RatioValue elementElementLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  elementElementLanePoint.lateralT = elementElementLanePointLateralT;
  ::ad::physics::Distance elementElementLanePointLaneLength(-1e9);
  elementElementLanePoint.laneLength = elementElementLanePointLaneLength;
  ::ad::physics::Distance elementElementLanePointLaneWidth(-1e9);
  elementElementLanePoint.laneWidth = elementElementLanePointLaneWidth;
  elementElement.lanePoint = elementElementLanePoint;
  ::ad::map::match::MapMatchedPositionType elementElementType(::ad::map::match::MapMatchedPositionType::INVALID);
  elementElement.type = elementElementType;
  ::ad::map::point::ECEFPoint elementElementMatchedPoint;
  ::ad::map::point::ECEFCoordinate elementElementMatchedPointX(-6400000);
  elementElementMatchedPoint.x = elementElementMatchedPointX;
  ::ad::map::point::ECEFCoordinate elementElementMatchedPointY(-6400000);
  elementElementMatchedPoint.y = elementElementMatchedPointY;
  ::ad::map::point::ECEFCoordinate elementElementMatchedPointZ(-6400000);
  elementElementMatchedPoint.z = elementElementMatchedPointZ;
  elementElement.matchedPoint = elementElementMatchedPoint;
  ::ad::physics::Probability elementElementProbability(0.);
  elementElement.probability = elementElementProbability;
  ::ad::map::point::ECEFPoint elementElementQueryPoint;
  ::ad::map::point::ECEFCoordinate elementElementQueryPointX(-6400000);
  elementElementQueryPoint.x = elementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate elementElementQueryPointY(-6400000);
  elementElementQueryPoint.y = elementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate elementElementQueryPointZ(-6400000);
  elementElementQueryPoint.z = elementElementQueryPointZ;
  elementElement.queryPoint = elementElementQueryPoint;
  ::ad::physics::Distance elementElementMatchedPointDistance(-1e9);
  elementElement.matchedPointDistance = elementElementMatchedPointDistance;
  element.resize(1, elementElement);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
