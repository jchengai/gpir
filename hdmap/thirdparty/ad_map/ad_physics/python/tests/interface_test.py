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

import ad_physics as ad


class AdPhysicsPythonTest(unittest.TestCase):

    """
    Test class for Python interface
    """

    def test_interface(self):
        """
        Main test part
        """
        speed_a = ad.physics.Speed(10.)
        speed_b = ad.physics.Speed(20.)
        speed_c = speed_a + speed_b

        self.assertEqual(speed_c, 30.)

        t = ad.physics.Duration(2.)
        a = ad.physics.Acceleration(10.)

        speed_c = t * a

        self.assertEqual(speed_c, 20.)

        t = speed_c / a
        self.assertEqual(t, 2.)


if __name__ == '__main__':
    if os.environ.get('GTEST_OUTPUT') and os.environ['GTEST_OUTPUT'].startswith('xml:'):
        base_folder = os.environ['GTEST_OUTPUT'][4:]
        result_filename = base_folder + 'ad_pyhsics_interface_test_python' + str(sys.version_info.major) + ".xml"
        with open(result_filename, "w+") as result_file:
            unittest.main(testRunner=xmlrunner.XMLTestRunner(output=result_file))
    else:
        unittest.main()
