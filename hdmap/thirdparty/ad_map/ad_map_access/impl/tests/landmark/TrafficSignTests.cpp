// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include "KnownTrafficSigns.hpp"
#include "LandmarkTestBase.hpp"

struct TrafficSignTests : public LandmarkTestBase
{
};

TEST_F(TrafficSignTests, generate_traffic_signs)
{
  access::Factory factory(access::getStore());

  for (std::size_t signIndex = 0u; signIndex < landmark::knownTrafficSigns.size(); ++signIndex)
  {
    landmark::LandmarkId landmarkId(100 + signIndex);
    ECEFPoint position
      = createECEFPoint(8.44 + static_cast<double>(signIndex) * 0.000002, 49.000002, 115.); // approx. every 2 meter
    ECEFPoint orientation = createECEFPoint(8.4399 + static_cast<double>(signIndex) * 0.000002, 49.000002, 115.);
    ECEFEdge boundingBoxGeometry{position, orientation};
    Geometry bounding_box = createGeometry(boundingBoxGeometry, true);
    EXPECT_TRUE(factory.addTrafficSign(mPartitionId,
                                       landmarkId,
                                       landmark::knownTrafficSigns[signIndex],
                                       position,
                                       orientation,
                                       bounding_box,
                                       std::to_string(signIndex)));
    EXPECT_TRUE(factory.add(mLaneId, landmarkId));
  }

  EXPECT_EQ(landmark::getLandmarks().size(), landmark::knownTrafficSigns.size());
  EXPECT_EQ(lane::getLanes().front(), mLaneId);
  auto lane = lane::getLane(mLaneId);
  EXPECT_EQ(lane.visibleLandmarks.size(), 73u);
  EXPECT_EQ(lane.visibleLandmarks.front(), landmark::LandmarkId(100));

  // write the map
  size_t version_major = 0;
  size_t version_minor = 0;
  serialize::SerializerFileCRC32 serializer(true);
  serializer.open("test_files/test_traffic_signs.adm", version_major, version_minor);
  access::getStore().save(serializer);
  serializer.close();
}
