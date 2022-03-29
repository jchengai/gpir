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

struct GeometryOperationTest : ::testing::Test
{
  GeometryOperationTest()
  {
  }

  virtual ~GeometryOperationTest() = default;

  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
    access::cleanup();
  }
};

TEST_F(GeometryOperationTest, NearestPointOnEdge)
{
  ECEFEdge pts;
  pts.push_back(createECEFPoint(2, 1, 0));
  pts.push_back(createECEFPoint(4, 1, 0));
  pts.push_back(createECEFPoint(6, 3, 0));
  pts.push_back(createECEFPoint(6, 5, 0));
  Geometry geo = createGeometry(pts, false);
  {
    ECEFPoint pt = createECEFPoint(6.5, 3.5, 0);
    auto t_geo = findNearestPointOnEdge(geo, pt);
    ECEFPoint pt_geo = getParametricPoint(geo, t_geo);
    auto t_seg = findNearestPointOnSegment(pt, pts[2], pts[3]);
    ECEFPoint pt_seg = vectorInterpolate(pts[2], pts[3], t_seg);
    ASSERT_EQ(pt_geo, pt_seg);
  }
  {
    ECEFPoint pt = createECEFPoint(1, 1, 1);
    auto t = findNearestPointOnEdge(geo, pt);
    ASSERT_EQ(t, physics::ParametricValue(0.));
  }
  {
    ECEFPoint pt = createECEFPoint(2, 7, 0);
    auto t = findNearestPointOnEdge(geo, pt);
    ASSERT_EQ(t, physics::ParametricValue(1.));
  }
}

TEST_F(GeometryOperationTest, Position)
{
  ECEFEdge edge_ecef1;
  ECEFEdge edge_ecef2;
  Geometry geo1;
  Geometry geo2;

  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);

  ASSERT_FALSE(isSuccessor(geo1, geo2));
  ASSERT_FALSE(isPredecessor(geo1, geo2));
  ASSERT_FALSE(haveSameStart(geo1, geo2));
  ASSERT_FALSE(haveSameEnd(geo1, geo2));

  edge_ecef1.push_back(createECEFPoint(1, 2, 0));
  edge_ecef2.push_back(createECEFPoint(3, 4, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  ASSERT_FALSE(isSuccessor(geo1, geo2));
  ASSERT_FALSE(isPredecessor(geo1, geo2));
  ASSERT_FALSE(haveSameStart(geo1, geo2));
  ASSERT_FALSE(haveSameEnd(geo1, geo2));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(createECEFPoint(1, 2, 0));
  edge_ecef1.push_back(createECEFPoint(3, 4, 0));
  edge_ecef2.push_back(createECEFPoint(1, 2, 0));
  edge_ecef2.push_back(createECEFPoint(4, 5, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  ASSERT_TRUE(isPredecessor(geo1, geo2));
  ASSERT_TRUE(haveSameStart(geo1, geo2));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(createECEFPoint(1, 2, 0));
  edge_ecef1.push_back(createECEFPoint(3, 4, 0));
  edge_ecef2.push_back(createECEFPoint(1, 2, 1));
  edge_ecef2.push_back(createECEFPoint(3, 4, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  ASSERT_TRUE(isSuccessor(geo1, geo2));
  ASSERT_TRUE(haveSameEnd(geo1, geo2));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef1.push_back(createECEFPoint(1, 2, 0));
  edge_ecef1.push_back(createECEFPoint(3, 4, 0));
  edge_ecef2.push_back(createECEFPoint(3, 4, 0));
  edge_ecef2.push_back(createECEFPoint(1, 2, 0));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  ASSERT_TRUE(isSuccessor(geo1, geo2));
  ASSERT_TRUE(isPredecessor(geo1, geo2));
}

TEST_F(GeometryOperationTest, ParametricRange)
{
  physics::ParametricRange trange;
  trange.minimum = physics::ParametricValue(0.2);
  trange.maximum = physics::ParametricValue(0.8);
  ECEFEdge edge_ecef1;
  edge_ecef1.push_back(createECEFPoint(1, 2, 3));
  edge_ecef1.push_back(createECEFPoint(2, 2, 3));
  edge_ecef1.push_back(createECEFPoint(3, 2, 3));
  ECEFEdge edge_ecef2;
  Geometry geo1, geo2;
  geo1 = createGeometry(edge_ecef1, false);

  ECEFEdge edge_ecef3;
  getParametricRange(geo1, trange, edge_ecef3, false);
  ASSERT_EQ(edge_ecef3.size(), 3u);
  ASSERT_EQ(edge_ecef3[0], createECEFPoint(1.4, 2, 3));
  ASSERT_EQ(edge_ecef3[1], createECEFPoint(2, 2, 3));
  ASSERT_EQ(edge_ecef3[2], createECEFPoint(2.6, 2, 3));

  edge_ecef3.clear();
  getParametricRange(geo1, trange, edge_ecef3, true);
  ASSERT_EQ(edge_ecef3.size(), 3u);
  ASSERT_EQ(edge_ecef3[0], createECEFPoint(2.6, 2, 3));
  ASSERT_EQ(edge_ecef3[1], createECEFPoint(2, 2, 3));
  ASSERT_EQ(edge_ecef3[2], createECEFPoint(1.4, 2, 3));

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef3.clear();
  edge_ecef1.push_back(createECEFPoint(10000, 200000, 300000));
  edge_ecef1.push_back(createECEFPoint(20000, 200000, 300000));
  edge_ecef1.push_back(createECEFPoint(30000, 200000, 300000));
  geo1 = createGeometry(edge_ecef1, false);
  getParametricRange(geo1, trange, edge_ecef3, false);
  GeoEdge edge_geo1;
  getParametricRange(geo1, trange, edge_geo1, false);
  ASSERT_EQ(edge_geo1.size(), 3u);
  GeoEdge edge_geo2;
  edge_geo2 = toGeo(edge_ecef3);
  ASSERT_EQ(edge_geo1[0], edge_geo2[0]);
  ASSERT_EQ(edge_geo1[1], edge_geo2[1]);
  ASSERT_EQ(edge_geo1[2], edge_geo2[2]);

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef3.clear();
  edge_ecef1.push_back(createECEFPoint(10000, 200000, 300000));
  edge_ecef1.push_back(createECEFPoint(20000, 200000, 300000));
  edge_ecef1.push_back(createECEFPoint(30000, 200000, 300000));
  geo1 = createGeometry(edge_ecef1, false);
  getParametricRange(geo1, trange, edge_ecef3, true);
  getParametricRange(geo1, trange, edge_geo1, true);
  ASSERT_EQ(edge_geo1.size(), 3u);
  edge_geo2 = toGeo(edge_ecef3);
  ASSERT_EQ(edge_geo1[0], edge_geo2[0]);
  ASSERT_EQ(edge_geo1[1], edge_geo2[1]);
  ASSERT_EQ(edge_geo1[2], edge_geo2[2]);

  edge_ecef1.clear();
  edge_ecef2.clear();
  edge_ecef3.clear();
  edge_ecef1.push_back(createECEFPoint(1, 2, 3));
  edge_ecef1.push_back(createECEFPoint(2, 2, 3));
  edge_ecef1.push_back(createECEFPoint(3, 2, 3));
  edge_ecef2.push_back(createECEFPoint(6, 6, 6));
  edge_ecef2.push_back(createECEFPoint(8, 6, 6));
  geo1 = createGeometry(edge_ecef1, false);
  geo2 = createGeometry(edge_ecef2, false);
  edge_ecef3 = getMiddleEdge(geo1, geo2);
  ASSERT_EQ(edge_ecef3.size(), 3u);
  ASSERT_EQ(edge_ecef3[0], createECEFPoint(3.5, 4, 4.5));
  ASSERT_EQ(edge_ecef3[1], createECEFPoint(4.5, 4, 4.5));
  ASSERT_EQ(edge_ecef3[2], createECEFPoint(5.5, 4, 4.5));
}

TEST_F(GeometryOperationTest, ParametricRangeENU)
{
  physics::ParametricRange trange;
  ECEFEdge edge_ecef1, edge_ecef2;
  Geometry geome;
  ENUEdge edge_enu;

  trange.minimum = physics::ParametricValue(0.2);
  trange.maximum = physics::ParametricValue(0.8);

  CoordinateTransform mCoordinateTransform;
  const Altitude h(123.456);  // Invented altitude.
  const Altitude dh(100.123); // Invented altitude difference.
  GeoPoint geo0, geo1, geo2, geo3;
  // Position of Intel Office
  geo1 = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), h);
  // Position of San Jose Fire Department Station 29
  geo2 = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h);
  // Position of San Jose Fire Department Station 29 + altitude difference
  geo3 = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h + dh);

  ECEFPoint ecef1, ecef2, ecef3;
  ecef1 = mCoordinateTransform.Geo2ECEF(geo1);
  ecef2 = mCoordinateTransform.Geo2ECEF(geo2);
  ecef3 = mCoordinateTransform.Geo2ECEF(geo3);

  edge_ecef1.push_back(ecef1);
  edge_ecef1.push_back(ecef2);
  edge_ecef1.push_back(ecef3);
  geome = createGeometry(edge_ecef1, false);

  mCoordinateTransform.setENUReferencePoint(geo1);
  ENUPoint enu1, enu2, enu3;
  enu1 = mCoordinateTransform.ECEF2ENU(ecef1);
  enu2 = mCoordinateTransform.ECEF2ENU(ecef2);
  enu3 = mCoordinateTransform.ECEF2ENU(ecef3);

#if SAFE_DATAYPES_THROW
  //@todo: recheck when generator supports some compiler define to make feature visible to the outside
  EXPECT_THROW(getParametricRange(geome, trange, edge_enu, false), std::out_of_range);
#endif
  ASSERT_TRUE(access::init("test_files/TPK.adm.txt"));
#if SAFE_DATAYPES_THROW
  //@todo: recheck when generator supports some compiler define to make feature visible to the outside
  EXPECT_THROW(getParametricRange(geome, trange, edge_enu, false), std::out_of_range);
#endif
  access::setENUReferencePoint(geo1);

  edge_enu.clear();
  getParametricRange(geome, trange, edge_ecef2, false);
  getParametricRange(geome, trange, edge_enu, false);
  physics::Distance ecef_edge_length = calcLength(edge_ecef2);
  physics::Distance enu_edge_length = calcLength(edge_enu);
  EXPECT_EQ(ecef_edge_length, enu_edge_length);

  edge_enu.clear();
  getParametricRange(geome, trange, edge_ecef2, true);
  getParametricRange(geome, trange, edge_enu, true);
  ecef_edge_length = calcLength(edge_ecef2);
  enu_edge_length = calcLength(edge_enu);
  EXPECT_EQ(ecef_edge_length, enu_edge_length);
}
