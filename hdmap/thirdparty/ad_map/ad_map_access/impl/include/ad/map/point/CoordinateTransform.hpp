// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <memory>
#include <vector>

#include "ad/map/point/Types.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace point */
namespace point {

/**
 * @brief Class that encapsulates various coordinate transformations.
 */
class CoordinateTransform {
 public:
  typedef std::shared_ptr<CoordinateTransform>
      Ptr;  ///< Smart pointer to the object.
  typedef std::shared_ptr<CoordinateTransform const>
      ConstPtr;  ///< Smart pointer to the object.

 public:  // Constructor/Destructor
  /**
   * Constructor. Initialize working variables.
   */
  CoordinateTransform();

  /**
   * Destructor. Does nothing.
   */
  ~CoordinateTransform();

 public:  // Basic Conversions
 public:  // ENU Reference Point Managment
  /**
   * @brief Set the reference point for the ENU coordinate system.
   *        Increases ENU reference point counter.
   * @param[in] enu_ref_point ENU reference point.
   */
  void setENUReferencePoint(const GeoPoint &enu_ref_point);

  /**
   * @return ENU Reference Point.
   */
  const GeoPoint &getENUReferencePoint() const;

  /**
   * @returns true if ENU conversion is possible.
   */
  bool isENUValid() const;

  /**
   * @return ENU Reference Point counter.
   *         To be used to mark what ENU system is used.
   */
  size_t getENURef() const;

 public:  // ENU/Geo Conversions
  /**
   * @brief Convert point between coordinate systems.
   * @param[in] pt Source point.
   * @returns Point in the target coordinate system.
   * \note
   * http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf
   */
  ENUPoint Geo2ENU(const GeoPoint &pt) const;

  /**
   * @brief Convert point between coordinate systems.
   * @param[in] pt Source point.
   * @returns Point in the target coordinate system.
   * \note
   * http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf
   */
  GeoPoint ENU2Geo(const ENUPoint &pt) const;

 public:  // ECEF/Geo Conversions
          /**
           * @brief Convert point between coordinate systems.
           * @param[in] pt Source point.
           * @returns Point in the target coordinate system.
           * \note Olson, D. K. (1996):
           *       Converting earth-Centered, Earth-Fixed Coordinates to Geodetic
           * Coordinates,         IEEE Transactions on Aerospace and Electronic Systems,
           * Vol.         32, No. 1, January 1996,         pp. 473-476.
           */
  ECEFPoint Geo2ECEF(const GeoPoint &pt) const;

  /**
   * @brief Convert point between coordinate systems.
   * @param[in] pt Source point.
   * @returns Point in the target coordinate system.
   * \note Olson, D. K. (1996):
   *       Converting earth-Centered, Earth-Fixed Coordinates to Geodetic
   * Coordinates, IEEE Transactions on Aerospace and Electronic Systems, Vol.
   * 32, No. 1, January 1996, pp. 473-476.
   */
  GeoPoint ECEF2Geo(const ECEFPoint &pt) const;

 public:  // ECEF/ENU Conversions
          /**
           * @brief Convert point between coordinate systems.
           * @param[in] pt Source point.
           * @returns Point in the target coordinate system.
           * \note
           * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
           */
  ECEFPoint ENU2ECEF(const ENUPoint &pt) const;

  /**
   * @brief Convert point between coordinate systems.
   * @param[in] pt Source point.
   * @returns Point in the target coordinate system.
   * \note
   * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
   */
  ENUPoint ECEF2ENU(const ECEFPoint &pt) const;

 public:  // Useful methods
  /**
   * @brief Calculates Earth Radius at specific latitude.
   * @param[in] lat Latitude of the position.
   * @returns Earth Radius at specific latitude.
   * \note https://en.wikipedia.org/wiki/Earth_radius
   */
  static physics::Distance WGS84_R(const Latitude &lat);

  /**
   * @brief Convert geodetic latitude to a geocentric latitude.
   * @param[in] lat Geodetic latitude.
   * @returns The angle measured from center of Earth between a point and the
   * equator. \note https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
   */
  static double geocentricLatitude(const Latitude &lat);

 public:  // Conversions
  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const GeoPoint &x, ENUPoint &y) const { y = Geo2ENU(x); }

  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const ENUPoint &x, GeoPoint &y) const { y = ENU2Geo(x); }

  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const GeoPoint &x, ECEFPoint &y) const { y = Geo2ECEF(x); }

  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const ECEFPoint &x, GeoPoint &y) const { y = ECEF2Geo(x); }

  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const ECEFPoint &x, ENUPoint &y) const { y = ECEF2ENU(x); }

  /**
   * @brief Convert point between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in] x Source point.
   * @param[out] y Target point.
   */
  void convert(const ENUPoint &x, ECEFPoint &y) const { y = ENU2ECEF(x); }

  /**
   * @brief Convert points between coordinate systems.
   * \tparam SourceC Source coordinate system.
   * \tparam TargetC Target coordinate system.
   * @param[in]  xs Source points.
   * @param[out] ys Target points.
   */
  template <typename SourceC, typename TargetC>
  void convert(const std::vector<SourceC> &xs, std::vector<TargetC> &ys) const {
    ys.clear();
    ys.reserve(xs.size());
    for (auto x : xs) {
      TargetC y;
      convert(x, y);
      ys.push_back(y);
    }
  }

 private:                     // ENU Conversion parameters
  size_t enu_ref_;            ///< ENU Reference System Counter.
  GeoPoint enu_ref_point_;    ///< ENU Reference Point.
  ECEFPoint ecef_ref_point_;  ///< Equivalent ECEF Reference Point.
  double enu_phi_;            ///< ENU computation temp value.
  double enu_lam_;            ///< ENU computation temp value.
  double enu_h_;              ///< ENU computation temp value.
  double enu_tmp1_;           ///< ENU computation temp value.
  double enu_tmp1_3_;         ///< ENU computation temp value.
  double enu_cp_;             ///< ENU computation temp value.
  double enu_sp_;             ///< ENU computation temp value.
  double enu_cp_2_;           ///< ENU computation temp value.
  double enu_sp_2_;           ///< ENU computation temp value.
  double ecef_enu_[9];        ///< ENU/ECEF conversion matrix.

 private:                                    // Constants
  static constexpr double PI = 3.141592654;  ///< PI
  static constexpr double A = 6378137.0;     ///< WGS-84 semi-major axis.
  static constexpr double B = 6356752.3;     ///< WGS-84 semi-minor axis.
  static constexpr double E2 = 6.6943799901377997e-3;  ///< WGS-84 Eccentricy.
  static constexpr double A1 = A * E2;         ///< Precalculated constant.
  static constexpr double A2 = A1 * A1;        ///< Precalculated constant.
  static constexpr double A3 = A1 * E2 * 0.5;  ///< Precalculated constant.
  static constexpr double A4 = 2.5 * A2;       ///< Precalculated constant.
  static constexpr double A5 = A1 + A3;        ///< Precalculated constant.
  static constexpr double A6 =
      9.9330562000986220e-1;  ///< Precalculated constant.

 private:                           // Static Data
  static size_t instance_counter_;  ///< This class instance counter.
                                    ///< Used to generate initial reference
                                    ///< point identifier.
};

}  // namespace point
}  // namespace map
}  // namespace ad
