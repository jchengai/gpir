// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <gtest/gtest.h>

#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/serialize/SerializerFileCRC32.hpp>

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;

struct LandmarkTestBase : public ::testing::Test
{
  virtual void SetUp()
  {
    access::cleanup();
    access::Store::Ptr storePtr(new access::Store());
    mFactory = std::shared_ptr<access::Factory>(new access::Factory(*storePtr));
    mFactory->set(access::TrafficType::RIGHT_HAND_TRAFFIC);

    mLaneId = addLane();
    ASSERT_TRUE(isValid(mLaneId));
    ASSERT_TRUE(access::init(storePtr));
    access::setENUReferencePoint(createGeoPoint(Longitude(8.4), Latitude(49.), Altitude(115.)));
  }

  virtual void TearDown()
  {
    access::cleanup();
  }

  lane::LaneId addLane()
  {
    static lane::LaneId currentLaneId(10);
    currentLaneId = currentLaneId + lane::LaneId(1);
    lane::LaneId laneId = currentLaneId;

    // create lane
    auto laneAddResult = mFactory->add(mPartitionId, laneId, lane::LaneType::NORMAL, lane::LaneDirection::POSITIVE);
    if (laneAddResult)
    {
      // left border
      GeoEdge leftBorderPoints;
      leftBorderPoints.push_back(createGeoPoint(Longitude(8.3), Latitude(49.2), Altitude(115.)));
      leftBorderPoints.push_back(createGeoPoint(Longitude(8.5), Latitude(49.2), Altitude(115.)));
      // right border
      GeoEdge rightBorderPoints;
      rightBorderPoints.push_back(createGeoPoint(Longitude(8.3), Latitude(49.), Altitude(115.)));
      rightBorderPoints.push_back(createGeoPoint(Longitude(8.5), Latitude(49.), Altitude(115.)));

      laneAddResult = mFactory->set(
        laneId, createGeometry(toECEF(leftBorderPoints), false), createGeometry(toECEF(rightBorderPoints), false));
    }

    if (!laneAddResult)
    {
      laneId = lane::LaneId();
    }

    return laneId;
  }

  void reinitFromWrittenMap(std::string const &fileName)
  {
    // write the map
    size_t version_major = 0;
    size_t version_minor = 0;
    serialize::SerializerFileCRC32 serializer(true);
    serializer.open(fileName, version_major, version_minor);
    access::getStore().save(serializer);
    serializer.close();

    // read the map
    access::Store::Ptr storePtr(new access::Store());
    serialize::SerializerFileCRC32 deserializer(false);
    EXPECT_TRUE(deserializer.open(fileName, version_major, version_minor));
    EXPECT_TRUE(storePtr->load(deserializer));
    EXPECT_TRUE(deserializer.close());

    // reinit access interface
    access::cleanup();
    EXPECT_TRUE(access::init(storePtr));
  }

  std::shared_ptr<access::Factory> mFactory;
  access::PartitionId mPartitionId{0};
  lane::LaneId mLaneId;
};
