// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>

#include <fstream>
#include <gtest/gtest.h>
#include <streambuf>
#include <string>

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;

struct AdMapAccessTest : ::testing::Test
{
  virtual void SetUp()
  {
    access::cleanup();
  }
  virtual void TearDown()
  {
    access::cleanup();
  }
};

TEST_F(AdMapAccessTest, read_map)
{
  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));

  point::Longitude validLon(8.4372);
  point::Latitude validLat(49.02);
  point::Altitude validAlt(0.);

  auto p = point::createGeoPoint(validLon, validLat, validAlt);
  EXPECT_NO_THROW(access::setENUReferencePoint(p));

  auto lanes = lane::getLanes();
  ASSERT_GT(lanes.size(), 0u);
}

TEST_F(AdMapAccessTest, initialize)
{
  ASSERT_FALSE_NO_LOG(access::init("invalid/non_existent.txt"));

  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));
  ASSERT_TRUE(access::isENUReferencePointSet());

  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));

  ASSERT_FALSE_NO_LOG(access::init("test_files/TPK_PFZ.adm.txt"));
}

TEST_F(AdMapAccessTest, initializeFromOpenDriveContent)
{
  std::ifstream mapFile("test_files/Town01.xodr");
  std::stringstream openDriveContentStream;

  openDriveContentStream << mapFile.rdbuf();
  std::string openDriveContent = openDriveContentStream.str();

  ASSERT_FALSE_NO_LOG(
    access::initFromOpenDriveContent(std::string(), 0.2, intersection::IntersectionType::TrafficLight));

  ASSERT_TRUE(access::initFromOpenDriveContent(openDriveContent, 0.2, intersection::IntersectionType::TrafficLight));

  // twice the same initialization works
  ASSERT_TRUE(access::initFromOpenDriveContent(openDriveContent, 0.2, intersection::IntersectionType::TrafficLight));

  std::ifstream mapFile3("test_files/Town03.xodr");
  openDriveContentStream.clear();
  openDriveContentStream << mapFile3.rdbuf();
  openDriveContent = openDriveContentStream.str();

  // but different content fails
  ASSERT_FALSE_NO_LOG(
    access::initFromOpenDriveContent(openDriveContent, 0.2, intersection::IntersectionType::TrafficLight));

  access::cleanup();

  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));
  ASSERT_FALSE_NO_LOG(
    access::initFromOpenDriveContent(openDriveContent, 0.2, intersection::IntersectionType::TrafficLight));

  access::cleanup();
  access::Store::Ptr mStorePtr;
  std::shared_ptr<access::Factory> pFactory;
  ASSERT_FALSE(access::init("test_files/bad/TPK.adm.bad0.txt"));
  ASSERT_FALSE(access::init("test_files/bad/TPK.adm.bad1.txt"));
  ASSERT_FALSE(access::init("test_files/bad/TPK.adm.bad2.txt"));
  ASSERT_FALSE(access::init("test_files/bad/TPK.adm.bad3.txt"));

  mStorePtr.reset(new access::Store());
  pFactory.reset(new access::Factory(*mStorePtr));

  ECEFEdge rightPoints, leftPoints;
  leftPoints.push_back(
    toECEF(createGeoPoint(point::Longitude(8.44933898), point::Latitude(49.00737633), point::Altitude(0.))));
  leftPoints.push_back(
    toECEF(createGeoPoint(point::Longitude(8.45006151), point::Latitude(49.00885163), point::Altitude(0.))));
  rightPoints.push_back(
    toECEF(createGeoPoint(point::Longitude(8.44937788), point::Latitude(49.00736837), point::Altitude(0.))));
  rightPoints.push_back(
    toECEF(createGeoPoint(point::Longitude(8.45010064), point::Latitude(49.00884320), point::Altitude(0.))));
  point::Geometry geo1, geo2;
  geo1 = point::createGeometry(leftPoints, false);
  geo2 = point::createGeometry(rightPoints, false);
  pFactory->set(access::TrafficType::RIGHT_HAND_TRAFFIC);
  pFactory->add(access::PartitionId(0), lane::LaneId(11), lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
  pFactory->set(lane::LaneId(11), geo1, geo2);

  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));
  access::Store::Ptr emptyStore;
  ASSERT_FALSE(access::init(emptyStore));
  ASSERT_FALSE(access::init(mStorePtr));
  access::cleanup();

  ASSERT_TRUE(access::init(mStorePtr));
  ASSERT_TRUE(access::init(mStorePtr));
  ASSERT_FALSE(access::init("test_files/TPK.adm.txt"));

  emptyStore.reset(new access::Store());
  ASSERT_FALSE(access::init(emptyStore));
}

TEST_F(AdMapAccessTest, readMap_no_reader)
{
  point::Longitude validLon(8.4372);
  point::Latitude validLat(49.02);
  point::Altitude validAlt(0.);

  // nothing initialized throws exception
  auto p = point::createGeoPoint(validLon, validLat, validAlt);
  access::setENUReferencePoint(p);
  EXPECT_EQ(access::getENUReferencePoint(), p);
}
