// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <../src/opendrive/DataTypeConversion.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/access/Store.hpp>
#include <ad/map/config/MapConfigFileHandler.hpp>
#include <ad/map/intersection/Intersection.hpp>
#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/opendrive/AdMapFactory.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/serialize/SerializerFileCRC32.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::opendrive;
using namespace ::ad::map::landmark;

struct OpenDriveAccessTests : ::testing::Test
{
  virtual void SetUp()
  {
    access::cleanup();
  }
  virtual void TearDown()
  {
    access::cleanup();
  }

  void checkEdgePoints(lane::LaneId laneId, point::ECEFEdge const &edge)
  {
    EXPECT_GE(edge.size(), 2u) << static_cast<uint64_t>(laneId);
    if (edge.size() > 2u)
    {
      for (auto pointIter = edge.begin(); pointIter != edge.end(); pointIter++)
      {
        auto nextPointIter = pointIter + 1;
        if (nextPointIter != edge.end())
        {
          auto deltaPoints = *pointIter - *nextPointIter;
          auto pointDistance = vectorLength(deltaPoints);
          EXPECT_NE(pointDistance, physics::Distance(0.)) << static_cast<uint64_t>(laneId) << " num: " << edge.size();
        }
      }
    }
    physics::ParametricRange trange;
    trange.minimum = physics::ParametricValue(0.);
    trange.maximum = physics::ParametricValue(1.);
    auto ecefs = point::getParametricRange(edge, trange);
    EXPECT_EQ(edge.size(), ecefs.size()) << static_cast<uint64_t>(laneId);
  }

  void checkEdgeContacts(lane::Lane const &lane)
  {
    if (lane::isRouteable(lane))
    {
      for (auto successorContact : lane::getContactLanes(lane, lane::ContactLocation::SUCCESSOR))
      {
        auto successorLane = lane::getLane(successorContact.toLane);
        if (successorLane.direction == lane.direction)
        {
          EXPECT_EQ(lane.edgeLeft.ecefEdge.back(), successorLane.edgeLeft.ecefEdge.front())
            << static_cast<uint64_t>(lane.id) << " succ: " << static_cast<uint64_t>(successorLane.id);
          EXPECT_EQ(lane.edgeRight.ecefEdge.back(), successorLane.edgeRight.ecefEdge.front())
            << static_cast<uint64_t>(lane.id) << " succ: " << static_cast<uint64_t>(successorLane.id);
        }
        else
        {
          EXPECT_EQ(lane.edgeLeft.ecefEdge.back(), successorLane.edgeRight.ecefEdge.back())
            << static_cast<uint64_t>(lane.id) << " succ: " << static_cast<uint64_t>(successorLane.id);
          EXPECT_EQ(lane.edgeRight.ecefEdge.back(), successorLane.edgeLeft.ecefEdge.back())
            << static_cast<uint64_t>(lane.id) << " succ: " << static_cast<uint64_t>(successorLane.id);
        }
      }
      for (auto predecessorContact : lane::getContactLanes(lane, lane::ContactLocation::PREDECESSOR))
      {
        auto predecessorLane = lane::getLane(predecessorContact.toLane);
        if (predecessorLane.direction == lane.direction)
        {
          EXPECT_EQ(lane.edgeLeft.ecefEdge.front(), predecessorLane.edgeLeft.ecefEdge.back())
            << static_cast<uint64_t>(lane.id) << " pre: " << static_cast<uint64_t>(predecessorLane.id);
          EXPECT_EQ(lane.edgeRight.ecefEdge.front(), predecessorLane.edgeRight.ecefEdge.back())
            << static_cast<uint64_t>(lane.id) << " pre: " << static_cast<uint64_t>(predecessorLane.id);
        }
        else
        {
          EXPECT_EQ(lane.edgeLeft.ecefEdge.front(), predecessorLane.edgeRight.ecefEdge.front())
            << static_cast<uint64_t>(lane.id) << " pre: " << static_cast<uint64_t>(predecessorLane.id);
          EXPECT_EQ(lane.edgeRight.ecefEdge.front(), predecessorLane.edgeLeft.ecefEdge.front())
            << static_cast<uint64_t>(lane.id) << " pre: " << static_cast<uint64_t>(predecessorLane.id);
        }
      }
      for (auto leftContact : lane::getContactLanes(lane, lane::ContactLocation::LEFT))
      {
        auto leftLane = lane::getLane(leftContact.toLane);
        EXPECT_EQ(lane.edgeLeft.ecefEdge.front(), leftLane.edgeRight.ecefEdge.front())
          << static_cast<uint64_t>(lane.id) << " left: " << static_cast<uint64_t>(leftLane.id);
        EXPECT_EQ(lane.edgeLeft.ecefEdge.back(), leftLane.edgeRight.ecefEdge.back())
          << static_cast<uint64_t>(lane.id) << " left: " << static_cast<uint64_t>(leftLane.id);
      }
      for (auto rightContact : lane::getContactLanes(lane, lane::ContactLocation::RIGHT))
      {
        auto rightLane = lane::getLane(rightContact.toLane);
        EXPECT_EQ(lane.edgeRight.ecefEdge.front(), rightLane.edgeLeft.ecefEdge.front())
          << static_cast<uint64_t>(lane.id) << " right: " << static_cast<uint64_t>(rightLane.id);
        EXPECT_EQ(lane.edgeRight.ecefEdge.back(), rightLane.edgeLeft.ecefEdge.back())
          << static_cast<uint64_t>(lane.id) << " right: " << static_cast<uint64_t>(rightLane.id);
      }
    }
  }

  void checkMapMatching(lane::Lane const &lane)
  {
    match::AdMapMatching mapMatching;

    auto ecefPoint0 = lane::getParametricPoint(lane, physics::ParametricValue(0.), physics::ParametricValue(.5));
    auto enuPoint0 = point::toENU(ecefPoint0);
    auto mapMatchedPositions0
      = mapMatching.getMapMatchedPositions(enuPoint0, physics::Distance(5.), physics::Probability(0.01));
    auto ecefPoint1 = lane::getParametricPoint(lane, physics::ParametricValue(1.), physics::ParametricValue(.5));
    auto enuPoint1 = point::toENU(ecefPoint1);
    auto mapMatchedPositions1
      = mapMatching.getMapMatchedPositions(enuPoint1, physics::Distance(5.), physics::Probability(0.01));
    auto contactLanes = lane::getContactLanes(lane,
                                              {lane::ContactLocation::LEFT,
                                               lane::ContactLocation::RIGHT,
                                               lane::ContactLocation::SUCCESSOR,
                                               lane::ContactLocation::PREDECESSOR});
    lane::LaneIdSet expectedLanes;
    expectedLanes.insert(lane.id);
    for (auto contactLane : contactLanes)
    {
      expectedLanes.insert(contactLane.toLane);
    }
    EXPECT_NE(expectedLanes.size(), 0u);
    for (auto const matchedPosition : mapMatchedPositions0)
    {
      expectedLanes.erase(matchedPosition.lanePoint.paraPoint.laneId);
    }
    for (auto const matchedPosition : mapMatchedPositions1)
    {
      expectedLanes.erase(matchedPosition.lanePoint.paraPoint.laneId);
    }

    EXPECT_EQ(expectedLanes.size(), 0u);

    mapMatching.addHeadingHint(point::createENUHeading(0.), access::getENUReferencePoint());
    for (auto paramLon = physics::ParametricValue(0.); paramLon <= physics::ParametricValue(1.);
         paramLon += physics::ParametricValue(0.1))
    {
      for (auto paramLat = physics::ParametricValue(0.); paramLat <= physics::ParametricValue(1.);
           paramLat += physics::ParametricValue(0.2))
      {
        auto ecefPoint = lane::getParametricPoint(lane, paramLon, paramLat);
        auto enuPoint = point::toENU(ecefPoint);
        EXPECT_NO_THROW(auto mapMatchedPositions = mapMatching.getMapMatchedPositions(
                          enuPoint, physics::Distance(5.), physics::Probability(0.01)));
      }
    }
  }
};

TEST_F(OpenDriveAccessTests, read_config)
{
  config::MapConfigFileHandler configHandler{};
  ASSERT_TRUE(configHandler.readConfig("test_files/Town01.txt"));
  ASSERT_TRUE(configHandler.isInitialized());
}

TEST_F(OpenDriveAccessTests, read_map)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));

  point::Longitude validLon(8.00);
  point::Latitude validLat(49.00);
  point::Altitude validAlt(0.);

  auto p = point::createGeoPoint(validLon, validLat, validAlt);
  access::setENUReferencePoint(p);

  auto lanes = lane::getLanes();
  ASSERT_GT(lanes.size(), 0u);

  // write map for convenience
  serialize::SerializerFileCRC32 serializer(true);
  size_t versionMajorWrite = ::ad::map::serialize::SerializerFileCRC32::VERSION_MAJOR;
  size_t versionMinorWrite = ::ad::map::serialize::SerializerFileCRC32::VERSION_MINOR;
  serializer.open("test_files/Town01.adm", versionMajorWrite, versionMinorWrite);
  access::getStore().save(serializer);
  serializer.close();
}

TEST_F(OpenDriveAccessTests, read_written_map)
{
  ASSERT_TRUE(access::init("test_files/Town01.adm.txt"));

  point::Longitude validLon(8.00);
  point::Latitude validLat(49.00);
  point::Altitude validAlt(0.);

  auto p = point::createGeoPoint(validLon, validLat, validAlt);

  access::setENUReferencePoint(p);

  auto lanes = lane::getLanes();
  ASSERT_GT(lanes.size(), 0u);
}

TEST_F(OpenDriveAccessTests, lane_points_town01)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgePoints(lane.id, lane.edgeLeft.ecefEdge);
    checkEdgePoints(lane.id, lane.edgeRight.ecefEdge);
  }
}

TEST_F(OpenDriveAccessTests, lane_contact_points_town01)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgeContacts(lane);
  }
}

TEST_F(OpenDriveAccessTests, lane_points_town03)
{
  ASSERT_TRUE(access::init("test_files/Town03.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgePoints(lane.id, lane.edgeLeft.ecefEdge);
    checkEdgePoints(lane.id, lane.edgeRight.ecefEdge);
  }
}

TEST_F(OpenDriveAccessTests, lane_contact_points_town03)
{
  ASSERT_TRUE(access::init("test_files/Town03.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgeContacts(lane);
  }
}

TEST_F(OpenDriveAccessTests, lane_points_town04)
{
  ASSERT_TRUE(access::init("test_files/Town04.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgePoints(lane.id, lane.edgeLeft.ecefEdge);
    checkEdgePoints(lane.id, lane.edgeRight.ecefEdge);
  }
}

TEST_F(OpenDriveAccessTests, lane_contact_points_town04)
{
  ASSERT_TRUE(access::init("test_files/Town04.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkEdgeContacts(lane);
  }
}

TEST_F(OpenDriveAccessTests, map_matching_town01)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));

  for (auto laneId : lane::getLanes())
  {
    auto lane = lane::getLane(laneId);
    checkMapMatching(lane);
  }
}

TEST_F(OpenDriveAccessTests, TrafficSignTest)
{
  ASSERT_TRUE(access::init("test_files/bad/Town01copy.txt"));
  LandmarkIdList idList;
  idList = getLandmarks();
  ASSERT_EQ(idList.size(), 3u);
}

TEST_F(OpenDriveAccessTests, defaultIntersectionType)
{
  ASSERT_TRUE(access::init("test_files/bad/Town01copyAllWayStop.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyCrosswalk.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyHasWay.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyPriorityToRight.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyPriorityToRightAndStraight.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyStop.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyTrafficLight.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyUnknown.txt"));
  access::cleanup();
  ASSERT_TRUE(access::init("test_files/bad/Town01copyYield.txt"));
  access::cleanup();
  ASSERT_FALSE(access::init("test_files/bad/Town01copyNotValid.txt"));
  access::cleanup();
}

TEST_F(OpenDriveAccessTests, branch)
{
  access::Store::Ptr mStorePtr;
  mStorePtr.reset(new access::Store());
  ad::map::opendrive::AdMapFactory factory(*mStorePtr);
  ASSERT_FALSE(
    factory.createAdMap(std::string("test_files/bad/NoneExists.xodr"), 2.0, intersection::IntersectionType::HasWay));

  access::cleanup();
  ASSERT_FALSE(
    factory.createAdMap(std::string("test_files/bad/TownEmpty.xodr"), 2.0, intersection::IntersectionType::HasWay));
}

TEST_F(OpenDriveAccessTests, DataTypeConversion)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));
  ASSERT_EQ(landmark::LandmarkType::TRAFFIC_LIGHT, toLandmarkType(1000001));
  ASSERT_EQ(landmark::LandmarkType::OTHER, toLandmarkType(1000003));
  ASSERT_EQ(landmark::LandmarkType::TRAFFIC_SIGN, toLandmarkType(101));
  ASSERT_EQ(landmark::LandmarkType::UNKNOWN, toLandmarkType(99));

  ASSERT_EQ(landmark::TrafficSignType::DANGER, toTrafficSignType(101, 0));
  ASSERT_EQ(landmark::TrafficSignType::DANGER, toTrafficSignType(102, 0));

  ASSERT_EQ(landmark::TrafficSignType::LANES_MERGING, toTrafficSignType(120, 0));

  ASSERT_EQ(landmark::TrafficSignType::CAUTION_PEDESTRIAN, toTrafficSignType(133, 0));

  ASSERT_EQ(landmark::TrafficSignType::CAUTION_BICYCLE, toTrafficSignType(138, 0));
  ASSERT_EQ(landmark::TrafficSignType::YIELD, toTrafficSignType(205, 0));
  ASSERT_EQ(landmark::TrafficSignType::STOP, toTrafficSignType(206, 0));

  ASSERT_EQ(landmark::TrafficSignType::ROUNDABOUT, toTrafficSignType(215, 0));
  ASSERT_EQ(landmark::TrafficSignType::PASS_RIGHT, toTrafficSignType(222, 0));

  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN, toTrafficSignType(250, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES, toTrafficSignType(251, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS, toTrafficSignType(253, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE, toTrafficSignType(254, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT, toTrafficSignType(263, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH, toTrafficSignType(264, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT, toTrafficSignType(265, 0));
  ASSERT_EQ(landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR, toTrafficSignType(267, 0));

  ASSERT_EQ(landmark::TrafficSignType::MAX_SPEED, toTrafficSignType(274, -1));
  ASSERT_EQ(landmark::TrafficSignType::SPEED_ZONE_30_BEGIN, toTrafficSignType(274, 1));
  ASSERT_EQ(landmark::TrafficSignType::SPEED_ZONE_30_END, toTrafficSignType(274, 2));
  ASSERT_EQ(landmark::TrafficSignType::MAX_SPEED, toTrafficSignType(274, 3));

  ASSERT_EQ(landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION, toTrafficSignType(301, 0));
  ASSERT_EQ(landmark::TrafficSignType::PRIORITY_WAY, toTrafficSignType(306, 0));
  ASSERT_EQ(landmark::TrafficSignType::CITY_BEGIN, toTrafficSignType(310, 0));
  ASSERT_EQ(landmark::TrafficSignType::CITY_END, toTrafficSignType(311, 0));
  ASSERT_EQ(landmark::TrafficSignType::MOTORVEHICLE_BEGIN, toTrafficSignType(331, 0));
  ASSERT_EQ(landmark::TrafficSignType::UNKNOWN, toTrafficSignType(332, 0));

  ASSERT_EQ(lane::ContactType::YIELD, toContactType(205));
  ASSERT_EQ(lane::ContactType::STOP, toContactType(206));
  ASSERT_EQ(lane::ContactType::RIGHT_OF_WAY, toContactType(301));
  ASSERT_EQ(lane::ContactType::RIGHT_OF_WAY, toContactType(306));
  ASSERT_EQ(lane::ContactType::TRAFFIC_LIGHT, toContactType(1000001));
  ASSERT_EQ(lane::ContactType::UNKNOWN, toContactType(1100001));

  ASSERT_EQ(landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN, toTrafficLightType(1000001, 0));
  ASSERT_EQ(landmark::TrafficLightType::PEDESTRIAN_RED_GREEN, toTrafficLightType(1000002, 0));
  ASSERT_EQ(landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN, toTrafficLightType(1000007, 0));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000008, 0));
  ASSERT_EQ(landmark::TrafficLightType::SOLID_RED_YELLOW, toTrafficLightType(1000009, 0));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000010, 0));
  ASSERT_EQ(landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN, toTrafficLightType(1000011, 10));
  ASSERT_EQ(landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN, toTrafficLightType(1000011, 20));
  ASSERT_EQ(landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN, toTrafficLightType(1000011, 30));
  ASSERT_EQ(landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN, toTrafficLightType(1000011, 40));
  ASSERT_EQ(landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN, toTrafficLightType(1000011, 50));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000011, 60));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000012, 0));
  ASSERT_EQ(landmark::TrafficLightType::BIKE_RED_GREEN, toTrafficLightType(1000013, 0));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000014, 0));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000015, 0));
  ASSERT_EQ(landmark::TrafficLightType::UNKNOWN, toTrafficLightType(1000016, 0));

  ::opendrive::SignalReference signalReference;
  signalReference.parametricPosition = 0.5;
  ASSERT_EQ(lane::ContactLocation::SUCCESSOR, toContactLocation(signalReference, true));
  signalReference.parametricPosition = 0.4;
  ASSERT_EQ(lane::ContactLocation::PREDECESSOR, toContactLocation(signalReference, true));
  signalReference.inLaneOrientation = true;
  ASSERT_EQ(lane::ContactLocation::SUCCESSOR, toContactLocation(signalReference, false));
  signalReference.inLaneOrientation = false;
  ASSERT_EQ(lane::ContactLocation::PREDECESSOR, toContactLocation(signalReference, false));
  ASSERT_EQ(landmark::LandmarkId(100), toLandmarkId((int)100));
}
