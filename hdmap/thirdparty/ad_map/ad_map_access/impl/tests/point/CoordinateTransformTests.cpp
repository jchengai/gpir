// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "gtest/gtest.h"
#include <ad/map/point/Operation.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>
#include "RandomGeometry.hpp"

using namespace ::ad::physics;
using namespace ::ad::map;
using namespace ::ad::map::point;

struct CoordianteTransformTest : ::testing::Test
{
  CoordianteTransformTest()
  {
  }

  virtual void SetUp()
  {
    access::cleanup();
    GeoPoint enu_origin = createGeoPoint(Longitude(-78.12), Latitude(43.21), Altitude(120));
    mCoordinateTransform.setENUReferencePoint(enu_origin);
  }

  virtual void TearDown()
  {
    access::cleanup();
  }

  template <typename TF1, typename TF2> void TestCT(const TF1 &tf1, const TF2 &tf2)
  {
    TF1 tf1_from_tf1(tf1);
    TF2 tf2_from_tf2;
    TF1 tf1_from_tf2;
    TF2 tf2_from_tf1;
    TF1 tf1_from_tf2_from_tf1;
    TF2 tf2_from_tf1_from_tf2;
    ASSERT_TRUE(isValid(tf1_from_tf1));
    ASSERT_FALSE(isValid(tf2_from_tf2, false));
    ASSERT_FALSE(isValid(tf1_from_tf2, false));
    ASSERT_FALSE(isValid(tf2_from_tf1, false));
    ASSERT_FALSE(isValid(tf1_from_tf2_from_tf1, false));
    ASSERT_FALSE(isValid(tf2_from_tf1_from_tf2, false));

    tf2_from_tf2 = tf2;
    ASSERT_TRUE(isValid(tf1_from_tf1));
    ASSERT_TRUE(isValid(tf2_from_tf2));

    mCoordinateTransform.convert(tf2, tf1_from_tf2);
    mCoordinateTransform.convert(tf1, tf2_from_tf1);
    ASSERT_TRUE(isValid(tf1_from_tf2));
    ASSERT_TRUE(isValid(tf2_from_tf1));

    mCoordinateTransform.convert(tf2_from_tf1, tf1_from_tf2_from_tf1);
    mCoordinateTransform.convert(tf1_from_tf2, tf2_from_tf1_from_tf2);
    ASSERT_TRUE(isValid(tf1_from_tf2_from_tf1));
    ASSERT_TRUE(isValid(tf2_from_tf1_from_tf2));

    // consider the actual distances of the points to be 1cm accurate
    const Distance epsilon(0.01);
    EXPECT_LE(distance(tf1, tf1_from_tf1), epsilon);
    EXPECT_LE(distance(tf1, tf1_from_tf2), epsilon);
    EXPECT_LE(distance(tf1, tf1_from_tf2_from_tf1), epsilon);

    EXPECT_LE(distance(tf2, tf2_from_tf2), epsilon);
    EXPECT_LE(distance(tf2, tf2_from_tf1), epsilon);
    EXPECT_LE(distance(tf2, tf2_from_tf1_from_tf2), epsilon);
  }

  template <typename TF1, typename TF2> void TestCT2(const TF1 &tf1, const TF2 &tf2)
  {
    TestCT(tf1, tf2);
    TestCT(tf2, tf1);
  }

  CoordinateTransform mCoordinateTransform;
};

TEST_F(CoordianteTransformTest, Geo2ENU)
{
  // Position of tree from http://www.dirsig.org/docs/new/coordinates.html
  GeoPoint geo = createGeoPoint(Longitude(-78.120123), Latitude(43.21009), Altitude(124));
  ENUPoint enu = createENUPoint(-10, 10, 4);
  TestCT2(geo, enu);
}

TEST_F(CoordianteTransformTest, ECEF2ENU)
{
  // Position of tree from http://www.dirsig.org/docs/new/coordinates.html
  ENUPoint enu = createENUPoint(-10, 10, 4);
  // ECEFPoint ecef(958506.011, -4556367.375, 4344627.164);  // Original values
  ECEFPoint ecef = createECEFPoint(958506.00937351841, -4556367.3723087218, 4344627.1621201029); // Exact
  TestCT2(ecef, enu);
}

TEST_F(CoordianteTransformTest, ECEFGeo)
{
  {
    // Position of Intel Office
    GeoPoint geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456));
    ECEFPoint ecef = createECEFPoint(-2683524.7374057802, -4305306.3074802049, 3852945.8669066750);
    TestCT2(geo, ecef);
  }

  {
    // Position of tree from http://www.dirsig.org/docs/new/coordinates.html
    GeoPoint geo = createGeoPoint(Longitude(-78.120123), Latitude(43.21009), Altitude(124));
    // ECEFPoint ecef = createECEFPoint(958506.011, -4556367.375, 4344627.164);  // Original values
    ECEFPoint ecef = createECEFPoint(958506.00937351841, -4556367.3723087218, 4344627.1621201029); // Exact
    TestCT2(geo, ecef);
  }
}

TEST_F(CoordianteTransformTest, Distances)
{
  {
    const Altitude h(123.456);  // Invented altitude.
    const Altitude dh(100.123); // Invented altitude difference.

    GeoPoint geo0;
    ASSERT_FALSE(isValid(geo0, false));
    // Position of Intel Office
    GeoPoint geo1 = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), h);
    // Position of San Jose Fire Department Station 29
    GeoPoint geo2 = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h);
    // Position of San Jose Fire Department Station 29 + altitude difference
    GeoPoint geo3 = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h + dh);

    ASSERT_TRUE(isValid(geo1));
    ASSERT_TRUE(isValid(geo2));
    ASSERT_TRUE(isValid(geo3));

    ECEFPoint ecef0;
    ASSERT_FALSE(isValid(ecef0, false));

    ECEFPoint ecef1 = mCoordinateTransform.Geo2ECEF(geo1);
    ECEFPoint ecef2 = mCoordinateTransform.Geo2ECEF(geo2);
    ECEFPoint ecef3 = mCoordinateTransform.Geo2ECEF(geo3);

    ASSERT_TRUE(isValid(ecef1));
    ASSERT_TRUE(isValid(ecef2));
    ASSERT_TRUE(isValid(ecef3));

    ENUPoint enu0 = mCoordinateTransform.ECEF2ENU(ecef1);
    ASSERT_FALSE(isValid(enu0, false));

    mCoordinateTransform.setENUReferencePoint(geo1);
    ENUPoint enu1 = mCoordinateTransform.ECEF2ENU(ecef1);
    ENUPoint enu2 = mCoordinateTransform.ECEF2ENU(ecef2);
    ENUPoint enu3 = mCoordinateTransform.ECEF2ENU(ecef3);
    ASSERT_TRUE(isValid(enu1));
    ASSERT_TRUE(isValid(enu2));
    ASSERT_TRUE(isValid(enu3));

    Distance d_geo_12 = distance(geo1, geo2);
    Distance d_enu_12 = distance(enu1, enu2);
    Distance d_ecef_12 = distance(ecef1, ecef2);
    Distance d_geo_13 = distance(geo1, geo3);
    Distance d_enu_13 = distance(enu1, enu3);
    Distance d_ecef_13 = distance(ecef1, ecef3);
    Distance d_geo_23 = distance(geo2, geo3);
    Distance d_enu_23 = distance(enu2, enu3);
    Distance d_ecef_23 = distance(ecef2, ecef3);

    ASSERT_EQ(d_geo_12, d_ecef_12);
    ASSERT_EQ(d_geo_13, d_ecef_13);
    ASSERT_EQ(d_geo_23, d_ecef_23);
    ASSERT_EQ(d_geo_12, d_enu_12);
    ASSERT_EQ(d_geo_13, d_enu_13);
    ASSERT_EQ(d_geo_23, d_enu_23);
    ASSERT_EQ(d_enu_12, d_ecef_12);
    ASSERT_EQ(d_enu_13, d_ecef_13);
    ASSERT_EQ(d_enu_23, d_ecef_23);
    ASSERT_EQ(d_geo_23, Distance(static_cast<double>(dh)));
    ASSERT_EQ(d_enu_23, Distance(static_cast<double>(dh)));
    ASSERT_EQ(d_ecef_23, Distance(static_cast<double>(dh)));
  }
}

TEST_F(CoordianteTransformTest, GetRef)
{
  CoordinateTransform coordinateTransform;
  // Position of Intel Office
  GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456));
  // Position of San Jose Fire Department Station 29
  GeoPoint pt2_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), Altitude(123.456));

  size_t enu_ref = 0;
  size_t result = 0;
  coordinateTransform.setENUReferencePoint(pt1_geo);
  enu_ref = coordinateTransform.getENURef();
  coordinateTransform.setENUReferencePoint(pt2_geo);
  result = coordinateTransform.getENURef();
  ASSERT_EQ(enu_ref + 1, result);
}

TEST_F(CoordianteTransformTest, Branch)
{
  CoordinateTransform coordinateTransform;

  // Position of Intel Office
  GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456));

  GeoPoint geo_pt_invalid;
  ECEFPoint ecef_pt_invalid;
  ENUPoint enu_pt_invalid;

  EXPECT_THROW_NO_LOG(coordinateTransform.Geo2ENU(geo_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ENU2Geo(enu_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.Geo2ECEF(geo_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ECEF2Geo(ecef_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ENU2ECEF(enu_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ECEF2ENU(ecef_pt_invalid), std::invalid_argument);

  EXPECT_THROW_NO_LOG(coordinateTransform.setENUReferencePoint(geo_pt_invalid), std::invalid_argument);

  EXPECT_NO_THROW(coordinateTransform.setENUReferencePoint(pt1_geo));
  // ensure the transformation is working
  auto enu_origin_transformed = coordinateTransform.Geo2ENU(pt1_geo);
  EXPECT_EQ(enu_origin_transformed, point::createENUPoint(0., 0., 0.));

  EXPECT_THROW_NO_LOG(coordinateTransform.Geo2ENU(geo_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ENU2Geo(enu_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ENU2ECEF(enu_pt_invalid), std::invalid_argument);
  EXPECT_THROW_NO_LOG(coordinateTransform.ECEF2ENU(ecef_pt_invalid), std::invalid_argument);
}

TEST_F(CoordianteTransformTest, ENUGeometry)
{
  // Position of Intel Office
  GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456));
  // Position of San Jose Fire Department Station 29
  GeoPoint pt2_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), Altitude(123.456));

  ECEFPoint pt1_ecef = toECEF(pt1_geo);
  EXPECT_TRUE(isValid(pt1_ecef));

  ECEFPoint pt2_ecef = toECEF(pt2_geo);
  EXPECT_TRUE(isValid(pt2_ecef));

  static constexpr size_t geo1_n = 200;
  static constexpr size_t geo2_n = 250;
  static constexpr size_t geo1_seed = 123;
  static constexpr size_t geo2_seed = 456;

  Geometry geo1 = randGeometry(pt1_ecef, geo1_n, geo1_seed);
  EXPECT_TRUE(isValid(geo1));

  Geometry geo2 = randGeometry(pt2_ecef, geo2_n, geo2_seed);
  EXPECT_TRUE(isValid(geo2));

  // no ENU reference set
  EXPECT_THROW_NO_LOG(toENU(geo1.ecefEdge), std::invalid_argument);
  EXPECT_THROW_NO_LOG(toENU(geo2.ecefEdge), std::invalid_argument);

  const ENUEdge geo1_cf1_enu1 = toENU(geo1.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo1_cf1_enu1));
  const ENUEdge geo2_cf1_enu1 = toENU(geo2.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo2_cf1_enu1));

  const ENUEdge geo1_cf1_enu2 = toENU(geo1.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo1_cf1_enu2));
  const ENUEdge geo2_cf1_enu2 = toENU(geo2.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo2_cf1_enu2));

  const ENUEdge geo1_cf1_enu1a = toENU(geo1.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo1_cf1_enu1a));
  const ENUEdge geo2_cf1_enu1a = toENU(geo2.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo2_cf1_enu1a));

  EXPECT_NE(geo1_cf1_enu1, geo1_cf1_enu2);
  EXPECT_EQ(geo1_cf1_enu1, geo1_cf1_enu1a);
  EXPECT_NE(geo2_cf1_enu1, geo2_cf1_enu2);
  EXPECT_EQ(geo2_cf1_enu1, geo2_cf1_enu1a);

  const ENUEdge geo1_cf2_enu2 = toENU(geo1.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo1_cf2_enu2));
  const ENUEdge geo2_cf2_enu2 = toENU(geo2.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo2_cf2_enu2));

  const ENUEdge geo1_cf2_enu1 = toENU(geo1.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo1_cf2_enu1));
  const ENUEdge geo2_cf2_enu1 = toENU(geo2.ecefEdge, pt1_geo);
  EXPECT_TRUE(isValid(geo2_cf2_enu1));

  const ENUEdge geo1_cf2_enu2a = toENU(geo1.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo1_cf2_enu2a));
  const ENUEdge geo2_cf2_enu2a = toENU(geo2.ecefEdge, pt2_geo);
  EXPECT_TRUE(isValid(geo2_cf2_enu2a));

  EXPECT_NE(geo1_cf2_enu1, geo1_cf2_enu2);
  EXPECT_EQ(geo1_cf2_enu2, geo1_cf2_enu2a);
  EXPECT_NE(geo2_cf2_enu1, geo2_cf2_enu2);
  EXPECT_EQ(geo2_cf2_enu2, geo2_cf2_enu2a);

  EXPECT_EQ(geo1_cf1_enu1, geo1_cf2_enu1);
  EXPECT_EQ(geo1_cf1_enu2, geo1_cf2_enu2);
  EXPECT_EQ(geo2_cf1_enu1, geo2_cf2_enu1);
  EXPECT_EQ(geo2_cf1_enu2, geo2_cf2_enu2);
}

TEST_F(CoordianteTransformTest, TOENU)
{
  // Position of Intel Office
  GeoPoint pt1_geo = createGeoPoint(Longitude(-121.935549), Latitude(37.401336), Altitude(123.456));
  // Position of San Jose Fire Department Station 29
  GeoPoint pt2_geo = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), Altitude(123.456));
  Distance geo_abs_length = distance(pt1_geo, pt2_geo);
  GeoEdge geo_edge;
  geo_edge.push_back(pt1_geo);
  geo_edge.push_back(pt2_geo);
  Distance geo_edge_length = calcLength(geo_edge);
  EXPECT_EQ(geo_abs_length, geo_edge_length);

  ECEFPoint pt1_ecef = toECEF(pt1_geo);
  EXPECT_TRUE(isValid(pt1_ecef));
  ECEFPoint pt2_ecef = toECEF(pt2_geo);
  EXPECT_TRUE(isValid(pt2_ecef));
  Distance ecef_abs_length = distance(pt1_ecef, pt2_ecef);
  ECEFEdge ecef_edge;
  ecef_edge.push_back(pt1_ecef);
  ecef_edge.push_back(pt2_ecef);
  Distance ecef_edge_length = calcLength(ecef_edge);
  EXPECT_EQ(ecef_abs_length, ecef_edge_length);

  EXPECT_EQ(geo_abs_length, ecef_abs_length);

  const Altitude h(123.456);  // Invented altitude.
  const Altitude dh(100.123); // Invented altitude difference.
  GeoPoint enu_origin = createGeoPoint(Longitude(-121.933886), Latitude(37.401112), h + dh);
  ENUPoint pt1_enu;
  ENUPoint pt2_enu;
  Distance enu_length;
  pt1_enu = toENU(pt1_ecef, enu_origin);
  pt2_enu = toENU(pt2_ecef, enu_origin);
  enu_length = distance(pt1_enu, pt2_enu);
  EXPECT_EQ(enu_length, ecef_abs_length);

  pt1_enu = toENU(pt1_geo, enu_origin);
  pt2_enu = toENU(pt2_geo, enu_origin);
  enu_length = distance(pt1_enu, pt2_enu);
  EXPECT_EQ(enu_length, ecef_abs_length);

  ENUEdge enu_edge;
  enu_edge.push_back(pt1_enu);
  enu_edge.push_back(pt2_enu);
  GeoEdge geo_edge_ret;
  geo_edge_ret = toGeo(enu_edge, enu_origin);
  Distance geo_from_enu_length = calcLength(geo_edge_ret);
  EXPECT_EQ(geo_from_enu_length, geo_edge_length);

  ECEFEdge ecef_edge_ret;
  ecef_edge_ret = toECEF(enu_edge, enu_origin);
  Distance ecef_from_enu_length = calcLength(ecef_edge_ret);
  EXPECT_EQ(ecef_from_enu_length, ecef_edge_length);

  ENUEdge enu_edge_ret;
  enu_edge_ret = toENU(geo_edge_ret, enu_origin);
  enu_length = calcLength(enu_edge_ret);
  EXPECT_EQ(enu_length, geo_edge_length);
}
