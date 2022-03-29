// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "FactoryTests.hpp"
#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/lane/ContactOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/point/PointOperation.hpp>
#include <ad/map/restriction/RestrictionOperation.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>
#include <gtest/gtest.h>
#include "../point/RandomGeometry.hpp"
#include "ad/map/point/BoundingSphereOperation.hpp"

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::access;
using namespace ::ad::map::point;

FactoryTest::FactoryTest()
{
}

void FactoryTest::SetUp()
{
  access::cleanup();
  mVehicle.type = restriction::RoadUserType::INVALID;
  mVehicle.passengers = restriction::PassengerCount(2);
  mVehicle.length = physics::Distance(4.4);
  mVehicle.width = physics::Distance(2.2);
  mVehicle.height = physics::Distance(3.3);
  mVehicle.weight = physics::Weight(3000.);
  mStorePtr.reset(new access::Store());
  pFactory.reset(new access::Factory(*mStorePtr));
  Fill();
}

void FactoryTest::TearDown()
{
  access::cleanup();
}

bool FactoryTest::Fill()
{
  bool ok = false;

  access::PartitionId p(0);

  ok = pFactory->set(access::TrafficType::RIGHT_HAND_TRAFFIC);

  ok = ok && pFactory->add(p, x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  ok = ok && pFactory->add(p, x12, lane::LaneType::SHOULDER, lane::LaneDirection::NEGATIVE);
  ok = ok && pFactory->add(p, x13, lane::LaneType::MULTI, lane::LaneDirection::BIDIRECTIONAL);

  edgeLeft11 = randGeometry(51, 11);
  edgeRight11 = randGeometry(edgeLeft11.ecefEdge.front(), 52, 12);
  ok = ok && pFactory->set(x11, edgeLeft11, edgeRight11);
  edgeLeft12 = randGeometry(edgeRight11.ecefEdge.front(), 53, 13);
  edgeRight12 = randGeometry(edgeLeft12.ecefEdge.front(), 54, 14);
  ok = ok && pFactory->set(x12, edgeLeft12, edgeRight12);
  edgeLeft13 = randGeometry(edgeRight12.ecefEdge.front(), 55, 15);
  edgeRight13 = randGeometry(edgeLeft13.ecefEdge.front(), 56, 16);
  ok = ok && pFactory->set(x13, edgeLeft13, edgeRight13);

  lane::ContactLocation succ(lane::ContactLocation::SUCCESSOR);
  lane::ContactLocation pred(lane::ContactLocation::PREDECESSOR);
  lane::ContactLocation left(lane::ContactLocation::LEFT);
  lane::ContactLocation righ(lane::ContactLocation::RIGHT);

  lane::ContactTypeList free({lane::ContactType::FREE});
  lane::ContactTypeList curb_up({lane::ContactType::FREE, lane::ContactType::CURB_UP});
  lane::ContactTypeList curb_down({lane::ContactType::FREE, lane::ContactType::CURB_DOWN});

  restriction::Restriction all_vehicles;
  all_vehicles.roadUserTypes
    = {restriction::RoadUserType::CAR, restriction::RoadUserType::BUS, restriction::RoadUserType::TRUCK};
  restriction::Restriction pedestrians;
  pedestrians.roadUserTypes = {restriction::RoadUserType::PEDESTRIAN};
  restriction::Restriction predestrian_and_bikes;
  predestrian_and_bikes.roadUserTypes = {restriction::RoadUserType::PEDESTRIAN, restriction::RoadUserType::BICYCLE};

  restriction::RestrictionList all_vehicles_list;
  all_vehicles_list.push_back(all_vehicles);

  restriction::RestrictionList pedestrian_list;
  pedestrian_list.push_back(pedestrians);

  restriction::RestrictionList pedestrian_and_bikes_list;
  pedestrian_and_bikes_list.push_back(predestrian_and_bikes);

  restriction::Restrictions all_vehicle_restrs;
  all_vehicle_restrs.conjunctions = all_vehicles_list;
  restriction::Restrictions pedestrian_restrs;
  pedestrian_restrs.conjunctions = pedestrian_list;
  restriction::Restrictions pedestrian_and_bikes_restrs;
  pedestrian_and_bikes_restrs.conjunctions = pedestrian_and_bikes_list;

  ok = ok && pFactory->set(x11, all_vehicle_restrs);
  ok = ok && pFactory->set(x12, pedestrian_restrs);
  ok = ok && pFactory->set(x13, pedestrian_and_bikes_restrs);

  ok = ok && pFactory->add(x11, x12, succ, free, all_vehicle_restrs);
  ok = ok && pFactory->add(x12, x11, pred, free, all_vehicle_restrs);
  ok = ok && pFactory->add(x12, x13, left, free, pedestrian_and_bikes_restrs);
  ok = ok && pFactory->add(x13, x12, righ, free, pedestrian_restrs);
  ok = ok && pFactory->add(x13, x11, succ, curb_up, all_vehicle_restrs);
  ok = ok && pFactory->add(x11, x13, pred, curb_down, all_vehicle_restrs);

  landmark::LandmarkId landmarkId(1234);
  point::ECEFPoint orientation = point::createECEFPoint(0., 0., 0.);
  point::ECEFPoint position = point::createECEFPoint(1., 0., 0.);
  point::Geometry bounding_box = point::createGeometry({orientation, position}, true);
  ok = ok && pFactory->addLandmark(p, landmarkId, landmark::LandmarkType::UNKNOWN, position, orientation, bounding_box);
  ok = ok && pFactory->add(x11, landmarkId);

  ok = ok && access::init(mStorePtr);
  return ok;
}

void FactoryTest::Check(const lane::LaneId &id, lane::LaneType type, lane::LaneDirection direction)
{
  auto lane = lane::getLanePtr(id);
  ASSERT_TRUE(static_cast<bool>(lane));
  ASSERT_EQ(lane->id, id);
  ASSERT_EQ(lane->type, type);
  ASSERT_EQ(lane->direction, direction);
}

void FactoryTest::Check(const lane::LaneId &id, point::Geometry const &edgeLeft, point::Geometry const &edgeRight)
{
  auto lane = lane::getLanePtr(id);
  ASSERT_TRUE(static_cast<bool>(lane));
  ASSERT_EQ(lane->edgeLeft, edgeLeft);
  ASSERT_EQ(lane->edgeRight, edgeRight);
  ASSERT_NE(lane->edgeLeft, edgeRight);
  ASSERT_NE(lane->edgeRight, edgeLeft);
}

void FactoryTest::Check(const lane::LaneId &id_from,
                        const lane::LaneId &id_to,
                        lane::ContactLocation location,
                        const lane::ContactTypeList &types)
{
  auto lane = lane::getLanePtr(id_from);
  ASSERT_TRUE(static_cast<bool>(lane));
  for (auto contact_lane : lane->contactLanes)
  {
    if (contact_lane.toLane == id_to)
    {
      ASSERT_EQ(contact_lane.location, location);
      ASSERT_EQ(contact_lane.types, types);
      return;
    }
  }
  ASSERT_TRUE(false);
}

bool FactoryTest::IsAccessOk(const lane::LaneId &id, restriction::RoadUserType road_user_type)
{
  auto lane = lane::getLanePtr(id);
  if (lane)
  {
    restriction::VehicleDescriptor vehicle = mVehicle;
    vehicle.type = road_user_type;
    return lane::isAccessOk(*lane, vehicle);
  }
  else
  {
    return false;
  }
}

bool FactoryTest::IsTransitionOk(const lane::LaneId &id_from,
                                 const lane::LaneId &id_to,
                                 lane::ContactLocation location,
                                 restriction::RoadUserType road_user_type)
{
  auto lane = lane::getLanePtr(id_from);
  if (lane)
  {
    for (auto contact_lane : lane->contactLanes)
    {
      if (contact_lane.toLane == id_to)
      {
        if (contact_lane.location != location)
        {
          return false;
        }
        restriction::VehicleDescriptor vehicle = mVehicle;
        vehicle.type = road_user_type;
        return lane::isAccessOk(contact_lane, vehicle);
      }
    }
  }
  return false;
}

TEST_F(FactoryTest, TestFactory)
{
  MapMetaData emptyMetaData;
  emptyMetaData.trafficType = TrafficType(99);
  ASSERT_FALSE(isValid(emptyMetaData));
  Check(x11, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  Check(x12, lane::LaneType::SHOULDER, lane::LaneDirection::NEGATIVE);
  Check(x13, lane::LaneType::MULTI, lane::LaneDirection::BIDIRECTIONAL);

  Check(x11, edgeLeft11, edgeRight11);
  Check(x12, edgeLeft12, edgeRight12);
  Check(x13, edgeLeft13, edgeRight13);

  lane::ContactLocation succ(lane::ContactLocation::SUCCESSOR);
  lane::ContactLocation pred(lane::ContactLocation::PREDECESSOR);
  lane::ContactLocation left(lane::ContactLocation::LEFT);
  lane::ContactLocation righ(lane::ContactLocation::RIGHT);

  lane::ContactTypeList free({lane::ContactType::FREE});
  lane::ContactTypeList curb_up({lane::ContactType::FREE, lane::ContactType::CURB_UP});
  lane::ContactTypeList curb_down({lane::ContactType::FREE, lane::ContactType::CURB_DOWN});

  Check(x11, x12, succ, free);
  Check(x12, x11, pred, free);
  Check(x12, x13, left, free);
  Check(x13, x12, righ, free);
  Check(x13, x11, succ, curb_up);
  Check(x11, x13, pred, curb_down);

  ASSERT_TRUE(IsAccessOk(x11, restriction::RoadUserType::BUS));
  ASSERT_TRUE(IsAccessOk(x11, restriction::RoadUserType::CAR));
  ASSERT_FALSE(IsAccessOk(x11, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_FALSE(IsAccessOk(x11, restriction::RoadUserType::BICYCLE));

  ASSERT_FALSE(IsAccessOk(x12, restriction::RoadUserType::BUS));
  ASSERT_FALSE(IsAccessOk(x12, restriction::RoadUserType::CAR));
  ASSERT_TRUE(IsAccessOk(x12, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_FALSE(IsAccessOk(x12, restriction::RoadUserType::BICYCLE));

  ASSERT_FALSE(IsAccessOk(x13, restriction::RoadUserType::BUS));
  ASSERT_FALSE(IsAccessOk(x13, restriction::RoadUserType::CAR));
  ASSERT_TRUE(IsAccessOk(x13, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_TRUE(IsAccessOk(x13, restriction::RoadUserType::BICYCLE));

  ASSERT_TRUE(IsTransitionOk(x11, x12, succ, restriction::RoadUserType::BUS));
  ASSERT_TRUE(IsTransitionOk(x11, x12, succ, restriction::RoadUserType::CAR));
  ASSERT_FALSE(IsTransitionOk(x11, x12, succ, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_FALSE(IsTransitionOk(x11, x12, succ, restriction::RoadUserType::BICYCLE));

  ASSERT_FALSE(IsTransitionOk(x12, x13, succ, restriction::RoadUserType::BUS));
  ASSERT_FALSE(IsTransitionOk(x12, x13, succ, restriction::RoadUserType::CAR));
  ASSERT_FALSE(IsTransitionOk(x12, x13, succ, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_FALSE(IsTransitionOk(x12, x13, succ, restriction::RoadUserType::BICYCLE));

  ASSERT_FALSE(IsTransitionOk(x12, x13, left, restriction::RoadUserType::BUS));
  ASSERT_FALSE(IsTransitionOk(x12, x13, left, restriction::RoadUserType::CAR));
  ASSERT_TRUE(IsTransitionOk(x12, x13, left, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_TRUE(IsTransitionOk(x12, x13, left, restriction::RoadUserType::BICYCLE));

  ASSERT_FALSE(IsTransitionOk(x13, x12, righ, restriction::RoadUserType::BUS));
  ASSERT_FALSE(IsTransitionOk(x13, x12, righ, restriction::RoadUserType::CAR));
  ASSERT_TRUE(IsTransitionOk(x13, x12, righ, restriction::RoadUserType::PEDESTRIAN));
  ASSERT_FALSE(IsTransitionOk(x13, x12, righ, restriction::RoadUserType::BICYCLE));

  auto landmarks = lane::getLane(x11).visibleLandmarks;
  ASSERT_EQ(landmarks.size(), 1u);
  ASSERT_TRUE(landmarks.front() == landmark::LandmarkId(1234));

  auto globalLandmarks = landmark::getLandmarks();
  ASSERT_EQ(globalLandmarks.size(), 1u);
  ASSERT_TRUE(landmarks.front() == globalLandmarks.front());

  access::PartitionId p(0);
  lane::LaneId retId;
  GeoEdge leftBorderPoints, rightBorderPoints;
  leftBorderPoints.push_back(createGeoPoint(Longitude(8.3), Latitude(49.2), Altitude(115.)));
  leftBorderPoints.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
  rightBorderPoints.push_back(createGeoPoint(Longitude(8.3), Latitude(49.), Altitude(115.)));
  rightBorderPoints.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));

  retId = pFactory->add(p, leftBorderPoints, rightBorderPoints);
  ASSERT_NE(lane::LaneId(), retId);

  ad::map::restriction::Restriction rs1;
  ad::map::restriction::Restriction rs2;
  rs1.roadUserTypes = {restriction::RoadUserType::CAR};
  rs2.roadUserTypes = {restriction::RoadUserType::PEDESTRIAN};
  ASSERT_FALSE_NO_LOG(pFactory->add(x100, rs1, true));
  ASSERT_TRUE(pFactory->add(x11, rs1, true));
  ASSERT_TRUE(pFactory->add(x11, rs2, false));

  ASSERT_FALSE_NO_LOG(pFactory->set(x100, lane::LaneType::SHOULDER));
  ASSERT_TRUE(pFactory->set(x11, lane::LaneType::SHOULDER));
  lane::ComplianceVersion version1 = 0x20190905;
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, version1));
  ASSERT_TRUE(pFactory->set(x11, version1));
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, lane::LaneDirection::BIDIRECTIONAL));
  ASSERT_TRUE(pFactory->set(x11, lane::LaneDirection::BIDIRECTIONAL));

  ad::physics::Speed speedLimitValue(1e3);
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, speedLimitValue));

  ASSERT_TRUE(pFactory->set(x11, globalLandmarks));

  ASSERT_TRUE(pFactory->deleteLane(x12));
  ASSERT_FALSE_NO_LOG(pFactory->deleteLane(x100));
  ASSERT_FALSE_NO_LOG(pFactory->deleteLane(xInValid));

  landmark::LandmarkId landMarkInvalid;
  ASSERT_TRUE(pFactory->add(x11, landMarkInvalid)); // bugs?

  landmark::LandmarkId landmarkId(1234);
  ASSERT_FALSE_NO_LOG(pFactory->add(x100, landmarkId));
  ASSERT_FALSE_NO_LOG(pFactory->deleteLandmark(landMarkInvalid));
  ASSERT_TRUE(pFactory->deleteLandmark(landmarkId));

  restriction::Restriction all_vehicles;
  all_vehicles.roadUserTypes
    = {restriction::RoadUserType::CAR, restriction::RoadUserType::BUS, restriction::RoadUserType::TRUCK};
  restriction::RestrictionList all_vehicles_list;
  all_vehicles_list.push_back(all_vehicles);
  restriction::Restrictions all_vehicle_restrs;
  all_vehicle_restrs.conjunctions = all_vehicles_list;
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, all_vehicle_restrs));

  ad::map::lane::ContactLaneList cLaneList;
  lane::ContactLane contactLane;
  contactLane.toLane = x12;
  contactLane.location = succ;
  contactLane.types = free;
  contactLane.restrictions = all_vehicle_restrs;
  contactLane.trafficLightId = ad::map::landmark::LandmarkId();
  cLaneList.push_back(contactLane);
  ASSERT_FALSE_NO_LOG(pFactory->add(x100, cLaneList));
  ASSERT_TRUE(pFactory->add(x11, cLaneList));
  ASSERT_FALSE_NO_LOG(pFactory->deleteContacts(xInValid, x12));
  ASSERT_TRUE(pFactory->deleteContacts(x11, x12));

  lane::LaneId x14{14}, x15{15}, x16{16};
  GeoEdge x14Left, x14Right, x15Left, x15Right, x16Left, x16Right;
  x14Left.push_back(createGeoPoint(Longitude(8.3), Latitude(49.2), Altitude(115.)));
  x14Left.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
  x14Right.push_back(createGeoPoint(Longitude(8.3), Latitude(49.), Altitude(115.)));
  x14Right.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));
  x15Left.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
  x15Left.push_back(createGeoPoint(Longitude(8.7), Latitude(49.2), Altitude(115.)));
  x15Right.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));
  x15Right.push_back(createGeoPoint(Longitude(8.7), Latitude(49.), Altitude(115.)));
  x16Left.push_back(createGeoPoint(Longitude(8.7), Latitude(49.2), Altitude(115.)));
  x16Left.push_back(createGeoPoint(Longitude(8.9), Latitude(49.2), Altitude(115.)));
  x16Right.push_back(createGeoPoint(Longitude(8.7), Latitude(49.), Altitude(115.)));
  x16Right.push_back(createGeoPoint(Longitude(8.9), Latitude(49.), Altitude(115.)));
  x14 = pFactory->add(p, x14Left, x14Right);
  ASSERT_NE(x14, lane::LaneId());
  x16 = pFactory->add(p, x16Left, x16Right);
  ASSERT_NE(x16, lane::LaneId());
  ASSERT_FALSE_NO_LOG(pFactory->autoConnect(xInValid, x14));
  ASSERT_FALSE_NO_LOG(pFactory->autoConnect(x100, x14));
  ASSERT_FALSE_NO_LOG(pFactory->autoConnect(x14, x100));
  ASSERT_FALSE(pFactory->autoConnect(x14, x16));
  lane::LaneId x17{17};
  GeoEdge x17Left, x17Right;
  x17Left.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
  x17Left.push_back(createGeoPoint(Longitude(8.9), Latitude(49.2), Altitude(115.)));
  x17Right.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));
  x17Right.push_back(createGeoPoint(Longitude(8.9), Latitude(49.), Altitude(115.)));
  x17 = pFactory->add(p, x17Left, x17Right);
  ASSERT_NE(x17, lane::LaneId());
  ASSERT_TRUE(pFactory->autoConnect(x16, x17));
  lane::LaneId x18{18};
  GeoEdge x18Left, x18Right;
  x18Left.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
  x18Left.push_back(createGeoPoint(Longitude(8.9), Latitude(49.2), Altitude(115.)));
  x18Right.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));
  x18Right.push_back(createGeoPoint(Longitude(8.9), Latitude(49.), Altitude(115.)));
  x18 = pFactory->add(p, x18Left, x18Right);
  ASSERT_NE(x18, lane::LaneId());
  ASSERT_TRUE(pFactory->autoConnect(x17, x18));

  point::Geometry leftGeo, rightGeo;
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, leftGeo, rightGeo));

  point::CoordinateTransform cf;
  point::ECEFEdge left_ecef;
  point::ECEFEdge right_ecef;
  cf.convert(x15Left, left_ecef);
  cf.convert(x15Right, right_ecef);
  EXPECT_THROW_NO_LOG(pFactory->add(p, left_ecef, right_ecef, x11, x12), std::runtime_error);
  retId = pFactory->add(p, left_ecef, right_ecef, x14, x16);
  ASSERT_NE(retId, lane::LaneId());

  restriction::SpeedLimit speedLimit;
  speedLimit.speedLimit = physics::Speed(50.);
  ASSERT_FALSE_NO_LOG(pFactory->add(x100, speedLimit));
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, physics::Speed(50.)));
  ASSERT_TRUE_NO_LOG(pFactory->add(x11, speedLimit));
  speedLimit.speedLimit = physics::Speed(80.);
  ASSERT_TRUE_NO_LOG(pFactory->add(x11, speedLimit));
  ASSERT_TRUE(pFactory->set(x11, physics::Speed(50.)));

  landmark::LandmarkIdList landMarkList;
  ASSERT_FALSE_NO_LOG(pFactory->set(x100, landMarkList));
}

TEST_F(FactoryTest, TestRestriction)
{
  restriction::Restriction all_vehicles;
  restriction::VehicleDescriptor vehicle;

  EXPECT_THROW_NO_LOG(isAccessOk(all_vehicles, vehicle), std::runtime_error);

  all_vehicles.roadUserTypes
    = {restriction::RoadUserType::CAR, restriction::RoadUserType::BUS, restriction::RoadUserType::TRUCK};
  all_vehicles.negated = true;
  vehicle = mVehicle;
  all_vehicles.passengersMin = restriction::PassengerCount(3);
  ASSERT_TRUE(isAccessOk(all_vehicles, vehicle));
  vehicle.passengers = restriction::PassengerCount(4);
  all_vehicles.roadUserTypes.clear();
  ASSERT_FALSE(isAccessOk(all_vehicles, vehicle));

  restriction::Restrictions roadRestrictions;
  ASSERT_TRUE(isAccessOk(roadRestrictions, vehicle));

  restriction::Restriction pedestrians;
  pedestrians.roadUserTypes = {restriction::RoadUserType::PEDESTRIAN};
  pedestrians.passengersMin = 5.;
  pedestrians.negated = true;
  roadRestrictions.disjunctions.push_back(pedestrians);
  ASSERT_TRUE(isAccessOk(roadRestrictions, vehicle));

  restriction::Restriction roadRestriction;
  roadRestriction.negated = false;
  roadRestriction.passengersMin = 4.;
  roadRestriction.roadUserTypes
    = {restriction::RoadUserType::CAR, restriction::RoadUserType::BUS, restriction::RoadUserType::TRUCK};
  roadRestrictions.conjunctions.push_back(roadRestriction);
  restriction::PassengerCount ret(5.);
  ASSERT_EQ(getHOV(roadRestrictions), ret);
  mStorePtr->removePartition(access::PartitionId(0));
  ASSERT_TRUE(mStorePtr->empty());
}

TEST_F(FactoryTest, Store)
{
  ASSERT_FALSE(mStorePtr->empty());
  auto laneIdList = mStorePtr->getLanes(access::PartitionId(0));
  ASSERT_EQ(laneIdList.size(), 3u);
  ASSERT_EQ(laneIdList[0], lane::LaneId(11));
  ASSERT_EQ(laneIdList[1], lane::LaneId(12));
  ASSERT_EQ(laneIdList[2], lane::LaneId(13));

  auto laneMarkIdList = mStorePtr->getLandmarks(access::PartitionId(0));
  ASSERT_EQ(laneMarkIdList.size(), 1u);
  ASSERT_EQ(laneMarkIdList[0], landmark::LandmarkId(1234));
}
TEST_F(FactoryTest, StoreLaneLength)
{
  access::cleanup();
  mStorePtr.reset(new access::Store());
  pFactory.reset(new access::Factory(*mStorePtr));

  point::Geometry geo1, geo2;
  point::ECEFEdge leftPoints;
  point::ECEFPoint basePoint = randECEFPoint();
  leftPoints.push_back(basePoint);
  leftPoints.push_back(basePoint + point::createECEFPoint(70., 0., 0.));
  geo1 = point::createGeometry(leftPoints, false);
  point::ECEFEdge rightPoints;
  rightPoints.push_back(basePoint + point::createECEFPoint(0., -10, 0.));
  rightPoints.push_back(basePoint + point::createECEFPoint(70., -10, 0.));
  geo2 = point::createGeometry(rightPoints, false);
  pFactory->set(access::TrafficType::RIGHT_HAND_TRAFFIC);

  pFactory->add(access::PartitionId(0), lane::LaneId(11), lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);

  pFactory->set(lane::LaneId(11), geo1, geo2);
  access::init(mStorePtr);
  calcBoundingSphere(geo1, geo2);
  auto ret = mStorePtr->getCumulativeLaneLength();
  ASSERT_DOUBLE_EQ(static_cast<double>(ret), 70.0);

  lane::LaneIdList laneIdList;
  laneIdList = mStorePtr->getLanes(std::string("abc"), false);
  ASSERT_EQ(laneIdList.size(), 0u);
  laneIdList = mStorePtr->getLanes(std::string("::ad::map::lane::LaneType::NORMAL"), false);
  ASSERT_EQ(laneIdList.size(), 1u);

  laneIdList = mStorePtr->getLanes(access::PartitionId(0), std::string("abc"), false);
  ASSERT_EQ(laneIdList.size(), 0u);
  laneIdList = mStorePtr->getLanes(access::PartitionId(0), std::string("::ad::map::lane::LaneType::NORMAL"), false);
  ASSERT_EQ(laneIdList.size(), 1u);
}
