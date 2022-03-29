// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace ::ad::map;

struct TrafficLightTests : ::testing::Test
{
  virtual void SetUp()
  {
    access::cleanup();
    ASSERT_TRUE(access::init("test_files/TPK_PFZ.adm.txt"));
  }

  virtual void TearDown()
  {
    access::cleanup();
  }
};

landmark::ENULandmark make_landmark(uint64_t id, double x, double y, double heading)
{
  landmark::ENULandmark landmark;
  landmark.id = landmark::LandmarkId(id);
  landmark.position.x = point::ENUCoordinate(x);
  landmark.position.y = point::ENUCoordinate(y);
  landmark.position.z = point::ENUCoordinate(3.);
  landmark.heading = point::ENUHeading(heading);
  return landmark;
}

TEST_F(TrafficLightTests, validate_trafficLights_access)
{
  double const maxError{0.01};

  auto const laneGeoPosition
    = point::createGeoPoint(point::Longitude(8.4365541), point::Latitude(49.0159383), point::Altitude(0.));
  auto const laneId = lane::uniqueLaneId(laneGeoPosition);
  access::setENUReferencePoint(laneGeoPosition);

  EXPECT_THROW(landmark::getVisibleTrafficLights(lane::LaneId(1000)), std::invalid_argument);

  auto const mapTrafficLights = landmark::getVisibleTrafficLights(laneId);

  std::vector<landmark::ENULandmark> expectedLandmarks{make_landmark(21, -21.4540, -40.1775, -1.1057),
                                                       make_landmark(22, -14.2869, -36.6020, -1.1057),
                                                       make_landmark(17, -14.7092, -16.3109, 0.5676),
                                                       make_landmark(18, -19.0267, -9.5788, 0.5676),
                                                       make_landmark(13, 166.9595, 119.0969, 0.7017),
                                                       make_landmark(14, 161.7771, 125.1920, 0.7017),
                                                       make_landmark(19, -49.2998, -37.6138, -2.6007),
                                                       make_landmark(20, -45.1647, -44.4588, -2.6007),
                                                       make_landmark(15, 157.9260, 109.0071, -2.4645),
                                                       make_landmark(16, 162.9555, 102.7860, -2.4645)};

  ASSERT_EQ(mapTrafficLights.size(), expectedLandmarks.size());

  for (auto mapTrafficLightId : mapTrafficLights)
  {
    auto const mapTrafficLight = getENULandmark(mapTrafficLightId);
    auto expectedIter = expectedLandmarks.begin();
    do
    {
      // we cannot compare the traffic light id's since these may change in the map, but the positions
      // have to be constant
      if ((std::fabs(static_cast<double>(expectedIter->position.x - mapTrafficLight.position.x)) <= maxError)
          && (std::fabs(static_cast<double>(expectedIter->position.y - mapTrafficLight.position.y)) <= maxError)
          && (std::fabs(static_cast<double>(expectedIter->position.z - mapTrafficLight.position.z)) <= maxError)
          && (fmod(static_cast<double>(expectedIter->heading - mapTrafficLight.heading), 2 * M_PI) <= maxError))
      {
        break;
      }
      expectedIter++;
    } while (expectedIter != expectedLandmarks.end());

    ASSERT_NE(expectedIter, expectedLandmarks.end());
    expectedLandmarks.erase(expectedIter);
  }
}
