// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/point/ENUOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/point/Transform.hpp>
#include <gtest/gtest.h>

double const epsilon = 0.00001;

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;

struct PointOperationTest : ::testing::Test
{
  PointOperationTest()
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

TEST_F(PointOperationTest, dot_product)
{
  std::vector<std::pair<double, std::pair<ENUPoint, ENUPoint>>> testVectors{
    std::make_pair(0., std::make_pair(createENUPoint(10., 0., 0.), createENUPoint(0., 10., 0.))),
    std::make_pair(0., std::make_pair(createENUPoint(10., 0., 0.), createENUPoint(0., 0., 10.))),
    std::make_pair(0., std::make_pair(createENUPoint(0., 10., 0.), createENUPoint(0., 0., 10.))),
    std::make_pair(0., std::make_pair(createENUPoint(0., 10., 0.), createENUPoint(10., 0., 0.))),
    std::make_pair(0., std::make_pair(createENUPoint(0., 0., 10.), createENUPoint(10., 0., 0.))),
    std::make_pair(0., std::make_pair(createENUPoint(0., 0., 10.), createENUPoint(0., 10., 0.))),
    std::make_pair(100., std::make_pair(createENUPoint(10., 0., 0.), createENUPoint(10., 0., 0.))),
    std::make_pair(100., std::make_pair(createENUPoint(0., 10., 0.), createENUPoint(0., 10., 0.))),
    std::make_pair(100., std::make_pair(createENUPoint(0., 0., 10.), createENUPoint(0., 0., 10.))),
    std::make_pair(-100., std::make_pair(createENUPoint(10., 0., 0.), createENUPoint(-10., 0., 0.))),
    std::make_pair(-100., std::make_pair(createENUPoint(0., 10., 0.), createENUPoint(0., -10., 0.))),
    std::make_pair(-100., std::make_pair(createENUPoint(0., 0., 10.), createENUPoint(0., 0., -10.))),
    std::make_pair(300., std::make_pair(createENUPoint(10., 10., 10.), createENUPoint(10., 10., 10.))),
    std::make_pair(100., std::make_pair(createENUPoint(-10., 10., 10.), createENUPoint(10., 10., 10.))),
    std::make_pair(100., std::make_pair(createENUPoint(10., -10., 10.), createENUPoint(10., 10., 10.))),
    std::make_pair(100., std::make_pair(createENUPoint(10., 10., -10.), createENUPoint(10., 10., 10.))),
    std::make_pair(-100., std::make_pair(createENUPoint(-10., -10., 10.), createENUPoint(10., 10., 10.))),
    std::make_pair(-100., std::make_pair(createENUPoint(10., -10., -10.), createENUPoint(10., 10., 10.))),
    std::make_pair(-100., std::make_pair(createENUPoint(-10., 10., -10.), createENUPoint(10., 10., 10.))),
    std::make_pair(-300., std::make_pair(createENUPoint(-10., -10., -10.), createENUPoint(10., 10., 10.)))};

  for (auto entry : testVectors)
  {
    auto result = vectorDotProduct(entry.second.first, entry.second.second);
    EXPECT_NEAR(entry.first, result, epsilon);
    result = entry.second.first * entry.second.second;
    EXPECT_NEAR(entry.first, result, epsilon);
  }
}

TEST_F(PointOperationTest, cross_product)
{
  std::vector<std::pair<ENUPoint, std::pair<ENUPoint, ENUPoint>>> testVectors{
    std::make_pair(createENUPoint(0., 0., 1.), std::make_pair(createENUPoint(1., 0., 0.), createENUPoint(0., 1., 0.))),
    std::make_pair(createENUPoint(1., 0., 0.), std::make_pair(createENUPoint(0., 1., 0.), createENUPoint(0., 0., 1.))),
    std::make_pair(createENUPoint(0., 1., 0.), std::make_pair(createENUPoint(0., 0., 1.), createENUPoint(1., 0., 0.))),
  };

  for (auto entry : testVectors)
  {
    auto result = vectorCrossProduct(entry.second.first, entry.second.second);
    EXPECT_EQ(entry.first, result);
    result = vectorCrossProduct(entry.second.second, entry.second.first);
    EXPECT_EQ(-1. * entry.first, result);
  }
}

TEST_F(PointOperationTest, enu_heading_normalization)
{
  std::vector<double> numbers{0., 22., 85.6, 90., 101., 130., 179., 179.99};
  for (auto number : numbers)
  {
    ENUHeading a = createENUHeading(degree2radians(number));
    EXPECT_EQ(a, ENUHeading(degree2radians(number)));

    ENUHeading b = createENUHeading(degree2radians(-number));
    EXPECT_EQ(b, ENUHeading(degree2radians(-number)));

    ENUHeading a1 = createENUHeading(degree2radians(number + 360.));
    EXPECT_EQ(a1, ENUHeading(degree2radians(number)));

    ENUHeading a2 = createENUHeading(degree2radians(number - 360.));
    EXPECT_EQ(a2, ENUHeading(degree2radians(number)));

    ENUHeading b1 = createENUHeading(degree2radians(-number + 360.));
    EXPECT_EQ(b1, ENUHeading(degree2radians(-number)));

    ENUHeading b2 = createENUHeading(degree2radians(-number - 360.));
    EXPECT_EQ(b2, ENUHeading(degree2radians(-number)));
  }

  ENUHeading d = createENUHeading(degree2radians(-180.));
  EXPECT_EQ(d, ENUHeading(degree2radians(180)));
}

TEST_F(PointOperationTest, enu_heading_construction)
{
  GeoPoint enuReferencePoint;
  enuReferencePoint.longitude = Longitude(8.4399515);
  enuReferencePoint.latitude = Latitude(49.0189305);
  enuReferencePoint.altitude = Altitude(0.);

  access::init("test_files/TPK.adm.txt");

  access::setENUReferencePoint(enuReferencePoint);
  ECEFPoint enuReferencePointEcef = toECEF(enuReferencePoint);

  std::vector<std::pair<ENUPoint, double>> pointPairs{std::make_pair(createENUPoint(1., 0., 0.), 0.),
                                                      std::make_pair(createENUPoint(0., 1., 0.), 90.),
                                                      std::make_pair(createENUPoint(-1., 0., 0.), 180.),
                                                      std::make_pair(createENUPoint(0., -1., 0.), -90.),
                                                      std::make_pair(createENUPoint(5., 0., 0.), 0.),
                                                      std::make_pair(createENUPoint(5., 5., 0.), 45.),
                                                      std::make_pair(createENUPoint(0., 5., 0.), 90.),
                                                      std::make_pair(createENUPoint(-5., 5., 0.), 135.),
                                                      std::make_pair(createENUPoint(-5., 0., 0.), 180.),
                                                      std::make_pair(createENUPoint(-5., -5., 0.), -135.),
                                                      std::make_pair(createENUPoint(0., -5., 0.), -90.),
                                                      std::make_pair(createENUPoint(5., -5., 0.), -45.)};

  for (auto pairElement : pointPairs)
  {
    ECEFHeading heading = createECEFHeading(enuReferencePointEcef, toECEF(pairElement.first));
    ENUHeading a = createENUHeading(heading, enuReferencePoint);
    EXPECT_EQ(a, ENUHeading(degree2radians(pairElement.second)));
  }
}

TEST_F(PointOperationTest, NearestPointOnLine)
{
  {
    ECEFPoint pt0 = createECEFPoint(1, 1, 0);
    ECEFPoint pt1 = createECEFPoint(3, 1, 0);
    ECEFPoint pt = createECEFPoint(2, 2, 0);
    auto t = findNearestPointOnEdge(pt, pt0, pt1);
    ASSERT_EQ(t, physics::RatioValue(0.5));
  }
}

TEST_F(PointOperationTest, NearestPointOnSegment)
{
  ECEFPoint pt0 = createECEFPoint(1, 1, 0);
  ECEFPoint pt1 = createECEFPoint(3, 1, 0);
  {
    ECEFPoint pt = createECEFPoint(2, 2, 0);
    auto t = findNearestPointOnSegment(pt, pt0, pt1);
    ASSERT_EQ(t, physics::ParametricValue(0.5));
  }
  {
    ECEFPoint pt = createECEFPoint(0, 2, 0);
    auto t = findNearestPointOnSegment(pt, pt0, pt1);
    ASSERT_EQ(t, physics::ParametricValue(0.));
  }
  {
    ECEFPoint pt = createECEFPoint(7, 3, 7);
    auto t = findNearestPointOnSegment(pt, pt0, pt1);
    ASSERT_EQ(t, physics::ParametricValue(1.));
  }
}

TEST_F(PointOperationTest, CalcENUDistance)
{
  ENUPoint pt1 = point::createENUPoint(1., 2., 3.);
  ENUPoint pt2 = point::createENUPoint(2., 3., 4.);
  ENUEdge edge;
  edge.push_back(pt1);
  edge.push_back(pt2);
  auto ret = point::calcLength(edge);
  ASSERT_DOUBLE_EQ(static_cast<double>(ret), std::sqrt(3.));
  ENUHeading heading = normalizeENUHeading(point::createENUHeading(M_PI_2));
  ASSERT_EQ(heading, createENUHeading(M_PI_2));
}
