/* ----------------------------------------------------------------------------

 * GTSAM_UNSTABLE Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     dllexport.h
 * @brief    Symbols for exporting classes and methods from DLLs
 * @author   Richard Roberts
 * @date     Mar 9, 2013
 */

// Macros for exporting DLL symbols on Windows
// Usage example:
// In header file:
//   class GTSAM_EXPORT MyClass { ... };
//   
// Results in the following declarations:
// When included while compiling the GTSAM library itself:
//   class __declspec(dllexport) MyClass { ... };
// When included while compiling other code against GTSAM:
//   class __declspec(dllimport) MyClass { ... };

#pragma once

// Whether GTSAM is compiled as static or DLL in windows. 
// This will be used to decide whether include __declspec(dllimport) or not in headers
#define BUILD_SHARED_LIBS

#ifdef _WIN32
#  ifndef BUILD_SHARED_LIBS
#    define GTSAM_UNSTABLE_EXPORT
#    define GTSAM_UNSTABLE_EXTERN_EXPORT extern
#  else
#    ifdef GTSAM_UNSTABLE_EXPORTS
#      define GTSAM_UNSTABLE_EXPORT __declspec(dllexport)
#      define GTSAM_UNSTABLE_EXTERN_EXPORT __declspec(dllexport) extern
#    else
#      define GTSAM_UNSTABLE_EXPORT __declspec(dllimport)
#      define GTSAM_UNSTABLE_EXTERN_EXPORT __declspec(dllimport)
#    endif
#  endif
#else
#ifdef __APPLE__
#  define GTSAM_UNSTABLE_EXPORT __attribute__((visibility("default")))
#  define GTSAM_UNSTABLE_EXTERN_EXPORT extern
#else
#  define GTSAM_UNSTABLE_EXPORT
#  define GTSAM_UNSTABLE_EXTERN_EXPORT extern
#endif
#endif

#undef BUILD_SHARED_LIBS

