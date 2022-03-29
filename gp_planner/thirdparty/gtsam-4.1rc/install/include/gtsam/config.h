/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    config.h
 * @brief   Settings and paths configured with CMake
 * @author  Richard Roberts
 */

#pragma once

// Library version
#define GTSAM_VERSION_MAJOR 4
#define GTSAM_VERSION_MINOR 1
#define GTSAM_VERSION_PATCH 0
#define GTSAM_VERSION_NUMERIC 40100
#define GTSAM_VERSION_STRING "4.1.0"

// Paths to example datasets distributed with GTSAM
#define GTSAM_SOURCE_TREE_DATASET_DIR "/home/cj/research/gpir_github/src/gpir/gp_planner/thirdparty/gtsam-4.1rc/examples/Data"
#define GTSAM_INSTALLED_DATASET_DIR "/gtsam_examples/Data"

// Whether GTSAM is compiled to use quaternions for Rot3 (otherwise uses rotation matrices)
/* #undef GTSAM_USE_QUATERNIONS */

// Whether GTSAM is compiled to use Pose3::EXPMAP as the default coordinates mode for Pose3's retract and localCoordinates (otherwise, Pose3::FIRST_ORDER will be used)
#define GTSAM_POSE3_EXPMAP

// Whether GTSAM is compiled to use Rot3::EXPMAP as the default coordinates mode for Rot3's retract and localCoordinates (otherwise, Pose3::CAYLEY will be used)
#ifndef GTSAM_USE_QUATERNIONS
	#define GTSAM_ROT3_EXPMAP
#endif

// Whether we are using TBB (if TBB was found and GTSAM_WITH_TBB is enabled in CMake)
#define GTSAM_USE_TBB

// Whether we are using a TBB version higher than 2020
#define TBB_GREATER_EQUAL_2020

// Whether we are using system-Eigen or our own patched version
/* #undef GTSAM_USE_SYSTEM_EIGEN */

// Whether Eigen will use MKL (if MKL was found and GTSAM_WITH_EIGEN_MKL is enabled in CMake)
/* #undef GTSAM_USE_EIGEN_MKL */
/* #undef EIGEN_USE_MKL_ALL */

// Whether Eigen with MKL will use OpenMP (if OpenMP was found, Eigen uses MKL, and GTSAM_WITH_EIGEN_MKL_OPENMP is enabled in CMake)
/* #undef GTSAM_USE_EIGEN_MKL_OPENMP */

// Eigen library version (needed to avoid mixing versions, which often leads
// to segfaults)
#define GTSAM_EIGEN_VERSION_WORLD 3
#define GTSAM_EIGEN_VERSION_MAJOR 3
#define GTSAM_EIGEN_VERSION_MINOR 7

// The default allocator to use
/* #undef GTSAM_ALLOCATOR_BOOSTPOOL */
#define GTSAM_ALLOCATOR_TBB
/* #undef GTSAM_ALLOCATOR_STL */

// Option for not throwing the CheiralityException for points that are behind a camera
#define GTSAM_THROW_CHEIRALITY_EXCEPTION

// Make sure dependent projects that want it can see deprecated functions
#define GTSAM_ALLOW_DEPRECATED_SINCE_V41

// Support Metis-based nested dissection
#define GTSAM_SUPPORT_NESTED_DISSECTION

// Support Metis-based nested dissection
#define GTSAM_TANGENT_PREINTEGRATION
