// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/point/Operation.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;

struct GeoOperationTest : ::testing::Test
{
  GeoOperationTest()
  {
  }

  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
    access::cleanup();
  }
};

TEST_F(GeoOperationTest, geoPointapproxAltitude)
{
  Longitude ul_lon(10.123456);
  Longitude lr_lon(10.234567);
  Latitude ul_lat(-20.22222);
  Latitude lr_lat(-20.33333);
  Altitude ul_alt(100);
  Altitude ur_alt(200);
  Altitude ll_alt(300);
  Altitude lr_alt(400);
  //
  GeoEdge pts;
  GeoPoint pt_ul = createGeoPoint(ul_lon, ul_lat, ul_alt);
  GeoPoint pt_ur = createGeoPoint(ul_lon, lr_lat, ur_alt);
  GeoPoint pt_ll = createGeoPoint(lr_lon, ul_lat, ll_alt);
  GeoPoint pt_lr = createGeoPoint(lr_lon, lr_lat, lr_alt);
  pts.push_back(pt_ul);
  pts.push_back(pt_ur);
  pts.push_back(pt_ll);
  pts.push_back(pt_lr);
  //
  {
    GeoPoint ptX = createGeoPoint(ul_lon, ul_lat, AltitudeUnknown);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, pt_ul);
  }
  {
    GeoPoint ptX = createGeoPoint(ul_lon, lr_lat, AltitudeUnknown);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, pt_ur);
  }
  {
    GeoPoint ptX = createGeoPoint(lr_lon, ul_lat, AltitudeUnknown);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, pt_ll);
  }
  {
    GeoPoint ptX = createGeoPoint(lr_lon, lr_lat, AltitudeUnknown);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, pt_lr);
  }
  {
    GeoPoint ptY = vectorInterpolate(pt_ul, pt_ur, physics::ParametricValue(0.3));
    GeoPoint ptX = zeroAltitude(ptY);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, ptY);
  }
  {
    GeoPoint ptY = vectorInterpolate(pt_ur, pt_lr, physics::ParametricValue(0.7));
    GeoPoint ptX = zeroAltitude(ptY);
    GeoPoint pt = approxAltitude(ptX, pts);
    ASSERT_EQ(pt, ptY);
  }
  {
    const Altitude h(123.456);  // Invented altitude.
    const Altitude dh(100.123); // Invented altitude difference.
    // Position of Intel Office
    GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), h);
    // Position of San Jose Fire Department Station 29 + altitude difference
    GeoPoint pt2_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h + dh);

    GeoEdge edge1_geo;
    edge1_geo.push_back(pt2_geo);
    GeoPoint ret;
    ret = approxAltitude(pt1_geo, edge1_geo);
    ASSERT_EQ(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), h + dh), ret);
  }
}

TEST_F(GeoOperationTest, Orientation)
{
  const Altitude h(123.456);  // Invented altitude.
  const Altitude dh(100.123); // Invented altitude difference.
  // Position of Intel Office
  GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), h);
  // Position of San Jose Fire Department Station 29
  GeoPoint pt2_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h);
  // Position of San Jose Fire Department Station 29 + altitude difference
  GeoPoint pt3_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h + dh);

  GeoEdge edge2_geo;
  GeoEdge edge3_geo;
  edge2_geo.push_back(pt1_geo);
  edge2_geo.push_back(pt2_geo);
  edge3_geo.push_back(pt1_geo);
  edge3_geo.push_back(pt3_geo);
  ASSERT_TRUE(haveSameOrientation(edge2_geo, edge3_geo));
}

TEST_F(GeoOperationTest, isOnTheLeft)
{
  GeoEdge edge2_geo;
  GeoEdge edge3_geo;
  const double diff = 3.534679;
  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549 - diff), Latitude(37.401336), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  ASSERT_TRUE(isOnTheLeft(edge2_geo, edge3_geo));

  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549 + diff), Latitude(37.401336), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  ASSERT_TRUE(isOnTheLeft(edge2_geo, edge3_geo));

  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 - diff), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549 + diff), Latitude(37.401336), Altitude(123.456)));
  ASSERT_FALSE(isOnTheLeft(edge2_geo, edge3_geo));

  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549 + diff), Latitude(37.401336), Altitude(123.456)));
  ASSERT_TRUE(isOnTheLeft(edge2_geo, edge3_geo));

  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 + diff), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549 + diff), Latitude(37.401336 + diff), Altitude(123.456)));
  ASSERT_TRUE(isOnTheLeft(edge2_geo, edge3_geo));

  edge2_geo.clear();
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336 - diff), Altitude(123.456)));
  edge2_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.clear();
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456)));
  edge3_geo.push_back(createGeoPoint(Longitude(-121.935549 + diff), Latitude(37.401336 + diff), Altitude(123.456)));
  ASSERT_FALSE(isOnTheLeft(edge2_geo, edge3_geo));
}
