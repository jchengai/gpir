#!/bin/env python

# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (c) 2019-2020 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

"""
Simple unittest module to ensure that the Python binding is functional
"""

import unittest
import xmlrunner
import sys
import os

import ad_map_access as ad


class AdMapAccessPythonTest(unittest.TestCase):

    """
    Test class for Python interface
    """

    def test_interface(self):
        """
        Main test part
        """
        self.assertTrue(ad.map.access.init("test_files/TPK.adm.txt"))

        # map loaded
        lanes = ad.map.lane.getLanes()
        self.assertEqual(len(lanes), 141)

        # map matching
        mapMatching = ad.map.match.AdMapMatching()
        geoPoint = ad.map.point.GeoPoint()
        geoPoint.longitude = ad.map.point.Longitude(8.4401803)
        geoPoint.latitude = ad.map.point.Latitude(49.0191987)
        geoPoint.altitude = ad.map.point.Altitude(0.)

        mapMatchingResults = mapMatching.getMapMatchedPositions(
            geoPoint, ad.physics.Distance(0.01), ad.physics.Probability(0.05))
        self.assertEqual(len(mapMatchingResults), 1)

        # route planning
        routingStart = mapMatchingResults[0].lanePoint.paraPoint
        routingEnd = ad.map.point.ParaPoint()
        routingEnd.laneId = routingStart.laneId
        routingEnd.parametricOffset = ad.physics.ParametricValue(0.0)

        routeResult = ad.map.route.planRoute(ad.map.route.createRoutingPoint(
            routingStart), ad.map.route.createRoutingPoint(routingEnd))
        routeLength = ad.map.route.calcLength(routeResult.roadSegments[0])
        self.assertEqual(int(float(routeLength)), 4)

        ad.map.access.cleanup()


if __name__ == '__main__':
    if os.environ.get('GTEST_OUTPUT') and os.environ['GTEST_OUTPUT'].startswith('xml:'):
        base_folder = os.environ['GTEST_OUTPUT'][4:]
        result_filename = base_folder + 'ad_map_access_interface_test_python' + str(sys.version_info.major) + ".xml"
        with open(result_filename, "w+") as result_file:
            unittest.main(testRunner=xmlrunner.XMLTestRunner(output=result_file))
    else:
        unittest.main()
