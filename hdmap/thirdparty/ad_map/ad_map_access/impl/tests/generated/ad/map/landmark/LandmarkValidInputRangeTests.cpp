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

#include "ad/map/landmark/LandmarkValidInputRange.hpp"

TEST(LandmarkValidInputRangeTests, testValidInputRange)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::landmark::LandmarkType invalidInitializedMember(static_cast<::ad::map::landmark::LandmarkType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTypeTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::landmark::LandmarkType invalidInitializedMember(static_cast<::ad::map::landmark::LandmarkType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangePositionTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(-6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.position = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangePositionTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.position = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeOrientationTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(-6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.orientation = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeOrientationTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.orientation = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeBoundingBoxTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(-1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.boundingBox = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeBoundingBoxTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.boundingBox = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTrafficLightTypeTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::landmark::TrafficLightType invalidInitializedMember(
    static_cast<::ad::map::landmark::TrafficLightType>(-1));
  value.trafficLightType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTrafficLightTypeTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::landmark::TrafficLightType invalidInitializedMember(
    static_cast<::ad::map::landmark::TrafficLightType>(-1));
  value.trafficLightType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTrafficSignTypeTooSmall)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value below input range minimum
  ::ad::map::landmark::TrafficSignType invalidInitializedMember(static_cast<::ad::map::landmark::TrafficSignType>(-1));
  value.trafficSignType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LandmarkValidInputRangeTests, testValidInputRangeTrafficSignTypeTooBig)
{
  ::ad::map::landmark::Landmark value;
  ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.id = valueId;
  ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valuePosition;
  ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
  valuePosition.x = valuePositionX;
  ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
  valuePosition.y = valuePositionY;
  ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
  valuePosition.z = valuePositionZ;
  value.position = valuePosition;
  ::ad::map::point::ECEFPoint valueOrientation;
  ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
  valueOrientation.x = valueOrientationX;
  ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
  valueOrientation.y = valueOrientationY;
  ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
  valueOrientation.z = valueOrientationZ;
  value.orientation = valueOrientation;
  ::ad::map::point::Geometry valueBoundingBox;
  bool valueBoundingBoxIsValid{true};
  valueBoundingBox.isValid = valueBoundingBoxIsValid;
  bool valueBoundingBoxIsClosed{true};
  valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
  ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
  valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
  valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
  valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
  valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
  valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
  ::ad::physics::Distance valueBoundingBoxLength(-1e9);
  valueBoundingBox.length = valueBoundingBoxLength;
  ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
  valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
  value.boundingBox = valueBoundingBox;
  std::string valueSupplementaryText{"min"};
  value.supplementaryText = valueSupplementaryText;
  ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
  value.trafficLightType = valueTrafficLightType;
  ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
  value.trafficSignType = valueTrafficSignType;

  // override member with data type value above input range maximum
  ::ad::map::landmark::TrafficSignType invalidInitializedMember(static_cast<::ad::map::landmark::TrafficSignType>(-1));
  value.trafficSignType = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
