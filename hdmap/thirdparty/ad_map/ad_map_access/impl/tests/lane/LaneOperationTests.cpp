// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/lane/BorderOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/route/LaneIntervalOperation.hpp>
#include <ad/map/serialize/SerializerFileCRC32.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>
#include <gtest/gtest.h>
#include "../../src/lane/LaneOperationPrivate.hpp"
#include "../point/RandomGeometry.hpp"

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;
using namespace ::ad::map::lane;

struct LaneOperationTest : ::testing::Test
{
  LaneOperationTest()
  {
  }

  virtual void SetUp()
  {
  }
  virtual void TearDown()
  {
    access::cleanup();
  }
  void setupSingleLongLane()
  {
    mStorePtr.reset(new access::Store());
    pFactory.reset(new access::Factory(*mStorePtr));
    // setup one single long lane
    point::GeoPoint bottomRight
      = point::createGeoPoint(point::Longitude(8.44937788), point::Latitude(49.00736837), point::Altitude(0.));
    point::GeoPoint upRight
      = point::createGeoPoint(point::Longitude(8.45010064), point::Latitude(49.00884320), point::Altitude(0.));
    point::GeoPoint bottomLeft
      = point::createGeoPoint(point::Longitude(8.44933898), point::Latitude(49.00737633), point::Altitude(0.));
    point::GeoPoint upLeft
      = point::createGeoPoint(point::Longitude(8.45006151), point::Latitude(49.00885163), point::Altitude(0.));

    point::ECEFEdge rightPoints;
    rightPoints.push_back(point::toECEF(bottomRight));
    rightPoints.push_back(point::toECEF(upRight));
    rightEdge = point::createGeometry(rightPoints, false);
    point::ECEFEdge leftPoints;
    leftPoints.push_back(point::toECEF(bottomLeft));
    leftPoints.push_back(point::toECEF(upLeft));
    leftEdge = point::createGeometry(leftPoints, false);
    pFactory->set(access::TrafficType::RIGHT_HAND_TRAFFIC);

    pFactory->add(access::PartitionId(0), laneId, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);

    pFactory->set(laneId, leftEdge, rightEdge);
    access::init(mStorePtr);
  }
  lane::LaneId laneId{4711};
  point::Geometry leftEdge;
  point::Geometry rightEdge;
  access::Store::Ptr mStorePtr;
  std::shared_ptr<access::Factory> pFactory;
};

TEST_F(LaneOperationTest, getLane)
{
  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));
  auto pos = point::createGeoPoint(point::Longitude(8.4398011), point::Latitude(49.0188015), point::Altitude(0.));
  auto bad = point::createGeoPoint(point::Longitude(9.), point::Latitude(9.), point::Altitude(0.));
  EXPECT_THROW(lane::uniqueLaneId(bad), std::runtime_error);
  lane::LaneId id = lane::uniqueLaneId(pos);
  auto lane = lane::getLane(id);
  EXPECT_EQ(id, lane.id);
}

TEST_F(LaneOperationTest, getPointsOfInterest)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));
  auto pos = point::createGeoPoint(point::Longitude(0.00193915337), point::Latitude(-0.00295), point::Altitude(0.));
  std::vector<config::PointOfInterest> poI;
  poI = access::getPointsOfInterest(pos, physics::Distance(1.0));
  ASSERT_EQ(poI.size(), 0u);
  poI = access::getPointsOfInterest(pos, physics::Distance(5.0));
  ASSERT_EQ(poI.size(), 1u);
  ASSERT_EQ(poI.front().name, std::string("T1"));
}

TEST_F(LaneOperationTest, getLaneHeading)
{
  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));

  std::vector<::point::ENUHeading> headings;
  std::vector<match::MapMatchedPosition> mapMatchedPositions;
  match::MapMatchedPosition position;
  position.lanePoint.paraPoint.laneId = lane::uniqueLaneId(
    point::createGeoPoint(point::Longitude(8.4398011), point::Latitude(49.0188015), point::Altitude(0.)));
  position.lanePoint.paraPoint.parametricOffset = physics::ParametricValue(0.5);
  position.lanePoint.lateralT = physics::RatioValue(0.5);
  position.matchedPoint = point::toECEF(
    point::createGeoPoint(point::Longitude(8.4401510), point::Latitude(49.0191792), point::Altitude(0.)));
  mapMatchedPositions.push_back(position);
  position.lanePoint.paraPoint.laneId = lane::uniqueLaneId(
    point::createGeoPoint(point::Longitude(8.4398226), point::Latitude(49.0187687), point::Altitude(0.)));
  mapMatchedPositions.push_back(position);

  for (auto mapMatchedPosition : mapMatchedPositions)
  {
    headings.push_back(lane::getLaneENUHeading(mapMatchedPosition));
  }
  ASSERT_EQ(2u, headings.size());
  // since the two lanes are in opposite direction, the heading should differ by PI
  ASSERT_NEAR(static_cast<double>(headings[0]) + M_PI, static_cast<double>(headings[1]), 0.1);
  headings.clear();
}

TEST_F(LaneOperationTest, findNearestPointOnEdge)
{
  setupSingleLongLane();
  auto lane = lane::getLane(laneId);
  // perform query
  point::GeoPoint requestPoint
    = point::createGeoPoint(point::Longitude(8.4493674), point::Latitude(49.0073880), point::Altitude(0.));

  match::MapMatchedPosition mmpos;
  EXPECT_TRUE(lane::findNearestPointOnLane(lane, point::toECEF(requestPoint), mmpos));
  EXPECT_EQ(mmpos.type, match::MapMatchedPositionType::LANE_IN);
  point::GeoPoint requestPointVeryClose
    = point::createGeoPoint(point::Longitude(8.4493662), point::Latitude(49.0073866), point::Altitude(0.));
  EXPECT_TRUE(lane::findNearestPointOnLane(lane, point::toECEF(requestPointVeryClose), mmpos));
  EXPECT_EQ(mmpos.type, match::MapMatchedPositionType::LANE_IN);
  point::GeoPoint requestPointVeryFar
    = point::createGeoPoint(point::Longitude(9.4493662), point::Latitude(49.0073866), point::Altitude(0.));
  EXPECT_TRUE(lane::findNearestPointOnLane(lane, point::toECEF(requestPointVeryFar), mmpos));
  EXPECT_EQ(mmpos.type, match::MapMatchedPositionType::LANE_RIGHT);
  // write map for convenience
  serialize::SerializerFileCRC32 serializer(true);
  size_t versionMajorWrite = ::ad::map::serialize::SerializerFileCRC32::VERSION_MAJOR;
  size_t versionMinorWrite = ::ad::map::serialize::SerializerFileCRC32::VERSION_MINOR;
  serializer.open("test_files/test_straight.adm", versionMajorWrite, versionMinorWrite);
  access::getStore().save(serializer);
  serializer.close();
}

TEST_F(LaneOperationTest, LaneRelation)
{
  setupSingleLongLane();
  ECEFEdge edge_ecef1, edge_ecef2, edge_ecef3, edge_ecef4;
  Geometry geo1, geo2, geo3, geo4;
  ECEFPoint basePoint = randECEFPoint();
  lane::LaneId x11, x12;
  access::PartitionId p(0);
  uint32_t i = 100;
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  ASSERT_TRUE(pFactory->set(x11, geo3, geo4));

  edge_ecef1.push_back(basePoint + createECEFPoint(1, 4, 0));
  edge_ecef1.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(1, 2, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 2, 0));
  geo1 = createGeometry(edge_ecef1, false);
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  ASSERT_TRUE(pFactory->set(x11, geo1, geo4));
  geo2 = createGeometry(edge_ecef2, false);
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  ASSERT_TRUE(pFactory->set(x11, geo3, geo1));

  edge_ecef1.clear();
  edge_ecef2.clear();
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  edge_ecef1.push_back(basePoint + createECEFPoint(1, 4, 0));
  edge_ecef1.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(1, 2, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 2, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  pFactory->set(x11, geo1, geo2);
  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef1.push_back(basePoint + createECEFPoint(7, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 2, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(7, 2, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPhysicalPredecessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));
  ASSERT_TRUE(isPyhsicalSuccessor(*(lane::getLanePtr(x12)), *(lane::getLanePtr(x11))));
  geo1 = createGeometry(edge_ecef2, false);
  geo2 = createGeometry(edge_ecef1, false);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPhysicalPredecessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));
  ASSERT_TRUE(isPyhsicalSuccessor(*(lane::getLanePtr(x12)), *(lane::getLanePtr(x11))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef1.push_back(basePoint + createECEFPoint(7, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 7, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isVanishingLaneStart(*(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(2, 4, 0));
  edge_ecef1.push_back(basePoint + createECEFPoint(8, 8, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 4, 0));
  edge_ecef2.push_back(basePoint + createECEFPoint(8, 8, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isVanishingLaneEnd(*(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(1, 4, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(4, 4, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(1, 2, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(4, 2, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);

  point::ParaPoint para1;
  para1.laneId = x12;
  para1.parametricOffset = physics::ParametricValue(0.5);
  ASSERT_EQ(calcWidth(x12, physics::ParametricValue(0.5)), physics::Distance(2.));
  ASSERT_EQ(calcWidth(para1), physics::Distance(2.));

  match::LaneOccupiedRegion occupiedRegion;
  occupiedRegion.laneId = x12;
  occupiedRegion.longitudinalRange.minimum = physics::ParametricValue(0.2);
  occupiedRegion.longitudinalRange.maximum = physics::ParametricValue(0.8);
  occupiedRegion.lateralRange.minimum = physics::ParametricValue(0.3);
  occupiedRegion.lateralRange.maximum = physics::ParametricValue(0.7);
  ASSERT_EQ(calcWidth(occupiedRegion), physics::Distance(2.) * 0.4);
  ASSERT_EQ(calcLength(occupiedRegion), calcLength(x12) * 0.6);

  ECEFPoint ecef_pt1;
  ecef_pt1 = getProjectedParametricPoint(
    *(lane::getLanePtr(x12)), physics::ParametricValue(0.5), physics::ParametricValue(0.5));
  ASSERT_EQ(ecef_pt1, basePoint + createECEFPoint(2.5, 3, 1));
  ASSERT_FALSE(isSuccessorOrPredecessor(x11, x12));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 4, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 4, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(14, 8, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x11, geo1, geo2);

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 12, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(14, 12, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPyhsicalSuccessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(14, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 12, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 12, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPyhsicalSuccessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 12, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 8, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPyhsicalSuccessor(*(lane::getLanePtr(x12)), *(lane::getLanePtr(x11))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 6, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(10, 12, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 7, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(10, 6, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(14, 7, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_FALSE(isPyhsicalSuccessor(*(lane::getLanePtr(x12)), *(lane::getLanePtr(x11))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(20, 4, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(20, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(24, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(20, 4, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(24, 8, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x11, geo1, geo2);

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(20, 4, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(24, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(20, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(24, 12, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPhysicalPredecessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef2.push_back(basePoint + createECEFPoint(20, 4, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(24, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(20, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(24, 12, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_TRUE(isPhysicalPredecessor(*(lane::getLanePtr(x11)), *(lane::getLanePtr(x12))));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(24, 8, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(24, 4, 1));
  edge_ecef1.push_back(basePoint + createECEFPoint(20, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(24, 8, 1));
  edge_ecef2.push_back(basePoint + createECEFPoint(20, 8, 1));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  ASSERT_FALSE(isPhysicalPredecessor(*(lane::getLanePtr(x12)), *(lane::getLanePtr(x11))));
}

TEST_F(LaneOperationTest, LanePoint)
{
  setupSingleLongLane();
  EXPECT_THROW_NO_LOG(lane::getLane(lane::LaneId(3811)), std::invalid_argument);

  lane::Lane badLane;
  physics::ParametricValue leftPara(0.1);
  physics::ParametricValue rightPara(0.1);
  ASSERT_FALSE(isValid(getParametricPoint(badLane, leftPara, rightPara)));
  auto lane = lane::getLane(laneId);

  ECEFEdge edge_ecef1, edge_ecef2, edge_ecef3, edge_ecef4;
  Geometry geo1, geo2, geo3, geo4;
  ECEFPoint basePoint = randECEFPoint();
  lane::LaneId x11, x12, x13;
  access::PartitionId p(0);
  uint32_t i = 100;
  ECEFPoint ecef_pt1;
  point::ParaPoint para1;

  CoordinateTransform mCoordinateTransform;
  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(10., 200., 300.));
  edge_ecef1.push_back(basePoint + createECEFPoint(20., 200., 300.));
  edge_ecef2.push_back(basePoint + createECEFPoint(10., 300., 300.));
  edge_ecef2.push_back(basePoint + createECEFPoint(20., 300., 300.));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x11, geo1, geo2);
  ecef_pt1 = getParametricPoint(*(lane::getLanePtr(x11)), physics::ParametricValue(0.5), physics::ParametricValue(0.5));
  ASSERT_EQ(ecef_pt1, basePoint + createECEFPoint(15., 250., 300.));

  GeoPoint geo_pt1 = toGeo(basePoint);
  ENUPoint enu1, enu2;
  access::setENUReferencePoint(geo_pt1);
  para1.laneId = x11;
  para1.parametricOffset = physics::ParametricValue(0.5);
  enu2 = map::lane::getENULanePoint(para1, physics::ParametricValue(0.5));
  mCoordinateTransform.setENUReferencePoint(access::getENUReferencePoint());
  enu1 = mCoordinateTransform.ECEF2ENU(ecef_pt1);
  ASSERT_EQ(enu1, enu2);

  point::ParaPoint para2;
  point::ENUHeading laneHeading;
  point::ENUHeading inverseLaneHeading;
  laneHeading = lane::getLaneENUHeading(para1);
  inverseLaneHeading = point::createENUHeading(M_PI_2);
  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(basePoint + createECEFPoint(20., 200., 300.));
  edge_ecef1.push_back(basePoint + createECEFPoint(30., 200., 300.));
  edge_ecef2.push_back(basePoint + createECEFPoint(20., 300., 300.));
  edge_ecef2.push_back(basePoint + createECEFPoint(30., 300., 300.));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  x12 = lane::LaneId(i++);
  pFactory->add(p, x12, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(x12, geo1, geo2);
  lane::ContactLocation left(lane::ContactLocation::LEFT);
  lane::ContactTypeList free({lane::ContactType::FREE});
  restriction::Restriction all_vehicles;
  all_vehicles.roadUserTypes
    = {restriction::RoadUserType::CAR, restriction::RoadUserType::BUS, restriction::RoadUserType::TRUCK};
  restriction::RestrictionList all_vehicles_list;
  all_vehicles_list.push_back(all_vehicles);
  restriction::Restrictions all_vehicle_restrs;
  all_vehicle_restrs.conjunctions = all_vehicles_list;
  ASSERT_TRUE(pFactory->add(x11, x12, left, free, all_vehicle_restrs));
  ASSERT_TRUE(projectPositionToLaneInHeadingDirection(para1, inverseLaneHeading, para2));

  ASSERT_FALSE(satisfiesFilter(*(lane::getLanePtr(x11)), std::string("abc"), false));

  ASSERT_FALSE(access::isLeftHandedTraffic());
  pFactory->set(access::TrafficType::LEFT_HAND_TRAFFIC);
  ASSERT_TRUE(access::isLeftHandedTraffic());
}

TEST_F(LaneOperationTest, LaneOperation)
{
  setupSingleLongLane();
  auto lane = lane::getLane(laneId);
  restriction::SpeedLimit speedLimit;
  speedLimit.speedLimit = physics::Speed(0.);
  ASSERT_TRUE(pFactory->add(laneId, speedLimit));

  physics::ParametricRange trange;
  trange.minimum = physics::ParametricValue(0.2);
  trange.maximum = physics::ParametricValue(0.7);
  auto ptrLane = lane::getLanePtr(laneId);
  physics::Speed maxSpeed(0);
  maxSpeed = getMaxSpeed(*ptrLane, trange);
  ASSERT_EQ(std::numeric_limits<physics::Speed>::max(), maxSpeed);

  ECEFEdge edge_ecef1, edge_ecef2, edge_ecef3, edge_ecef4;
  Geometry geo1, geo2, geo3, geo4;
  lane::LaneId x11, x12;
  access::PartitionId p(0);
  uint32_t i = 100;
  x11 = lane::LaneId(i++);
  pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  edge_ecef1.push_back(createECEFPoint(1, 4, 0));
  edge_ecef1.push_back(createECEFPoint(401, 4, 0));
  edge_ecef2.push_back(createECEFPoint(1, 2, 0));
  edge_ecef2.push_back(createECEFPoint(401, 2, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  pFactory->set(x11, geo1, geo2);
  speedLimit.speedLimit = physics::Speed(100.);
  ASSERT_TRUE(pFactory->add(x11, speedLimit));

  physics::Duration duration;
  duration = getDuration(*(lane::getLanePtr(x11)), trange);
  ASSERT_EQ(duration, physics::Duration(2.));
  route::LaneInterval laneInt1;
  laneInt1.laneId = x11;
  laneInt1.start = ::ad::physics::ParametricValue(0.2);
  laneInt1.end = ::ad::physics::ParametricValue(0.7);
  ASSERT_EQ(calcDuration(laneInt1), physics::Duration(2.));
}

TEST_F(LaneOperationTest, BorderOperation)
{
  ENUEdge edge_enu1, edge_enu2, edge_enu3, edge_enu4;
  ENUBorder border_enu1, border_enu2, border_enu3;

  edge_enu1.push_back(createENUPoint(1, 4, 0));
  edge_enu1.push_back(createENUPoint(4, 4, 0));
  edge_enu2.push_back(createENUPoint(1, 2, 0));
  edge_enu2.push_back(createENUPoint(4, 2, 0));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  normalizeBorder(border_enu1, &border_enu2);
  ASSERT_EQ(border_enu1.right[1], createENUPoint(4, 2, 0));

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1., 4., 0));
  edge_enu1.push_back(createENUPoint(401., 4., 0));
  edge_enu1.push_back(createENUPoint(701., 4., 0));
  edge_enu2.push_back(createENUPoint(1., 2., 0));
  edge_enu2.push_back(createENUPoint(701., 2., 0));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  normalizeBorder(border_enu1, &border_enu2);
  ASSERT_EQ(border_enu1.right.size(), 3u);
  ASSERT_EQ(border_enu1.right[2], createENUPoint(701., 2., 0));

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(201., 4., 0));
  edge_enu1.push_back(createENUPoint(401., 4., 0));
  edge_enu1.push_back(createENUPoint(701., 4., 0));
  edge_enu2.push_back(createENUPoint(201., 2., 0));
  edge_enu2.push_back(createENUPoint(701., 2., 0));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1, 4, 0));
  edge_enu1.push_back(createENUPoint(101, 4, 0));
  edge_enu2.push_back(createENUPoint(1, 2, 0));
  edge_enu2.push_back(createENUPoint(101, 2, 0));
  border_enu2.left = edge_enu1;
  border_enu2.right = edge_enu2;
  normalizeBorder(border_enu1, &border_enu2);
  ASSERT_EQ(border_enu1.right.size(), 3u);
  ASSERT_EQ(border_enu1.right[2], createENUPoint(701., 2., 0));

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1, 4, 1));
  edge_enu1.push_back(createENUPoint(4, 4, 1));
  edge_enu2.push_back(createENUPoint(1, 2, 1));
  edge_enu2.push_back(createENUPoint(4, 2, 1));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  edge_enu3 = getLateralAlignmentEdge(border_enu1, physics::ParametricValue(0.5));
  edge_enu4.clear();
  edge_enu4.push_back(createENUPoint(1, 3, 1));
  edge_enu4.push_back(createENUPoint(4, 3, 1));
  ASSERT_EQ(edge_enu3, edge_enu4);

  edge_enu3.clear();
  edge_enu3.push_back(createENUPoint(1, 2, 1));
  edge_enu3.push_back(createENUPoint(101, 2, 1));
  physics::Distance dis = getDistanceEnuPointToLateralAlignmentEdge(createENUPoint(101, 202, 1), edge_enu3);
  ASSERT_EQ(dis, physics::Distance(200));

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu3.clear();
  edge_enu1.push_back(createENUPoint(1., 4, 1));
  edge_enu1.push_back(createENUPoint(401., 4, 1));
  edge_enu2.push_back(createENUPoint(501., 401, 1));
  edge_enu2.push_back(createENUPoint(901., 401, 1));
  edge_enu3 = edge_enu1;
  makeTransitionFromFirstEdgeContinuous(edge_enu1, edge_enu2);
  ASSERT_EQ(edge_enu1.size(), 3u);
  ASSERT_EQ(edge_enu1[0], edge_enu3[0]);
  ASSERT_NE(edge_enu1[1], edge_enu3[1]);
  ASSERT_GT(calcLength(edge_enu1), calcLength(edge_enu3));
  dis = calcLength(edge_enu3);

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu3.clear();
  edge_enu1.push_back(createENUPoint(1., 4, 1));
  edge_enu1.push_back(createENUPoint(401., 4, 1));
  edge_enu2.push_back(createENUPoint(501., 901, 1));
  edge_enu2.push_back(createENUPoint(901., 901, 1));
  makeTransitionFromFirstEdgeContinuous(edge_enu1, edge_enu2);
  ASSERT_GT(calcLength(edge_enu1), dis);

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu3.clear();
  edge_enu1.push_back(createENUPoint(1., 4, 1));
  edge_enu1.push_back(createENUPoint(401., 4, 1));
  edge_enu2.push_back(createENUPoint(501., 401, 1));
  edge_enu2.push_back(createENUPoint(901., 401, 1));
  edge_enu3 = edge_enu2;
  makeTransitionToSecondEdgeContinuous(edge_enu1, edge_enu2);
  ASSERT_EQ(edge_enu2.size(), 3u);
  ASSERT_EQ(edge_enu2[0], edge_enu1[1]);
  ASSERT_NE(edge_enu2[1], edge_enu3[0]);
  ASSERT_EQ(edge_enu2[2], edge_enu3[1]);
  ASSERT_GT(calcLength(edge_enu2), calcLength(edge_enu3));
  dis = calcLength(edge_enu2);

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu3.clear();
  edge_enu1.push_back(createENUPoint(1., 4, 1));
  edge_enu1.push_back(createENUPoint(401., 4, 1));
  edge_enu2.push_back(createENUPoint(501., 901, 1));
  edge_enu2.push_back(createENUPoint(901., 901, 1));
  makeTransitionToSecondEdgeContinuous(edge_enu1, edge_enu2);
  ASSERT_GT(calcLength(edge_enu2), dis);

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1., 404., 1));
  edge_enu1.push_back(createENUPoint(401., 404., 1));
  edge_enu2.push_back(createENUPoint(1., 4., 1));
  edge_enu2.push_back(createENUPoint(401., 4., 1));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(801., 604., 1));
  edge_enu1.push_back(createENUPoint(1201., 604., 1));
  edge_enu2.push_back(createENUPoint(801., 204., 1));
  edge_enu2.push_back(createENUPoint(1201., 204., 1));
  border_enu2.left = edge_enu1;
  border_enu2.right = edge_enu2;
  border_enu3 = border_enu1;
  makeTransitionFromFirstBorderContinuous(border_enu1, border_enu2);
  ASSERT_NE(border_enu1, border_enu3);
  ASSERT_EQ(border_enu1.left.size(), 4u);
  ASSERT_EQ(border_enu1.right.size(), 4u);
  ASSERT_EQ(border_enu1.left[0], border_enu3.left[0]);
  ASSERT_EQ(border_enu1.left[3], border_enu2.left[0]);
  ASSERT_EQ(border_enu1.right[0], border_enu3.right[0]);
  ASSERT_EQ(border_enu1.right[3], border_enu2.right[0]);
  ASSERT_GT(calcLength(border_enu1.left), calcLength(border_enu3.left));
  ASSERT_GT(calcLength(border_enu1.right), calcLength(border_enu3.right));
  physics::Distance leftedge_length = calcLength(border_enu1.left);
  physics::Distance rightedge_length = calcLength(border_enu1.right);

  auto borderLength = calcLength(border_enu1);
  ASSERT_EQ(0.5 * (leftedge_length + rightedge_length), borderLength);

  lane::ENUBorderList enuBorderList{border_enu1, border_enu1, border_enu1};
  ASSERT_EQ(3.0 * borderLength, calcLength(enuBorderList));

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1., 404., 1));
  edge_enu1.push_back(createENUPoint(401., 404., 1));
  edge_enu2.push_back(createENUPoint(1., 4., 1));
  edge_enu2.push_back(createENUPoint(401., 4., 1));
  border_enu1.left = edge_enu1;
  border_enu1.right = edge_enu2;
  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(801., 1004., 1));
  edge_enu1.push_back(createENUPoint(1201., 1004., 1));
  edge_enu2.push_back(createENUPoint(801., 604., 1));
  edge_enu2.push_back(createENUPoint(1201., 604., 1));
  border_enu2.left = edge_enu1;
  border_enu2.right = edge_enu2;
  border_enu3 = border_enu1;
  makeTransitionFromFirstBorderContinuous(border_enu1, border_enu2);
  ASSERT_GT(calcLength(border_enu1.left), leftedge_length);
  ASSERT_GT(calcLength(border_enu1.right), rightedge_length);

  struct IndexPairs indexPairs = getIndexPairs(edge_enu1, edge_enu2);
  ASSERT_EQ(indexPairs.leftEdgeIndices.size(), 2u);
  ASSERT_EQ(indexPairs.leftEdgeIndices[0], 0u);
  ASSERT_EQ(indexPairs.leftEdgeIndices[1], 1u);
  ASSERT_EQ(indexPairs.leftEdgeIndices, indexPairs.rightEdgeIndices);

  edge_enu1.clear();
  edge_enu2.clear();
  edge_enu1.push_back(createENUPoint(1., 404., 1));
  edge_enu1.push_back(createENUPoint(401., 404., 1));
  edge_enu1.push_back(createENUPoint(801., 404., 1));
  edge_enu2.push_back(createENUPoint(1., 4., 1));
  edge_enu2.push_back(createENUPoint(401., 4., 1));
  indexPairs = getIndexPairs(edge_enu1, edge_enu2);
  ASSERT_EQ(indexPairs.leftEdgeIndices.size(), 3u);
  ASSERT_EQ(indexPairs.leftEdgeIndices[0], 0u);
  ASSERT_EQ(indexPairs.leftEdgeIndices[1], 1u);
  ASSERT_EQ(indexPairs.leftEdgeIndices[2], 2u);
  ASSERT_EQ(indexPairs.rightEdgeIndices.size(), 3u);
  ASSERT_EQ(indexPairs.rightEdgeIndices[0], 0u);
  ASSERT_EQ(indexPairs.rightEdgeIndices[1], 1u);
  ASSERT_EQ(indexPairs.rightEdgeIndices[2], 1u);
}
