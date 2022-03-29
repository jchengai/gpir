// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/point/CoordinateTransform.hpp"
#include "ad/map/access/Logging.hpp"

#include <cmath>
#include <cstring>
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace point {

//////////////////////////
// Constructor/Destructor

CoordinateTransform::CoordinateTransform()
{
  enu_ref_ = instance_counter_;
  instance_counter_ += 0x00100000;
  enu_h_ = 0;
  enu_tmp1_ = 0;
  enu_tmp1_3_ = 0;
  enu_cp_ = 0;
  enu_sp_ = 0;
  enu_cp_2_ = 0;
  enu_sp_2_ = 0;
  enu_phi_ = 0;
  enu_lam_ = 0;
  std::memset(ecef_enu_, 0, sizeof(ecef_enu_));
}

CoordinateTransform::~CoordinateTransform()
{
}

void CoordinateTransform::setENUReferencePoint(const GeoPoint &enu_ref_point)
{
  if (isValid(enu_ref_point))
  {
    enu_ref_++;
    enu_ref_point_ = enu_ref_point;
    ecef_ref_point_ = Geo2ECEF(enu_ref_point);

    enu_phi_ = toRadians(enu_ref_point_.latitude);
    enu_lam_ = toRadians(enu_ref_point_.longitude);
    double sin_lam = std::sin(enu_lam_);
    double cos_lam = std::cos(enu_lam_);
    double sin_phi = std::sin(enu_phi_);
    double cos_phi = std::cos(enu_phi_);

    enu_h_ = static_cast<double>(enu_ref_point_.altitude);
    enu_tmp1_ = std::sqrt(1 - E2 * sin_phi * sin_phi);
    enu_tmp1_3_ = enu_tmp1_ * enu_tmp1_ * enu_tmp1_;
    enu_cp_ = cos_phi;
    enu_sp_ = sin_phi;
    enu_cp_2_ = enu_cp_ * enu_cp_;
    enu_sp_2_ = enu_sp_ * enu_sp_;

    ecef_enu_[0] = -sin_lam;
    ecef_enu_[1] = cos_lam;
    ecef_enu_[2] = 0;
    ecef_enu_[3] = -cos_lam * sin_phi;
    ecef_enu_[4] = -sin_lam * sin_phi;
    ecef_enu_[5] = cos_phi;
    ecef_enu_[6] = cos_lam * cos_phi;
    ecef_enu_[7] = sin_lam * cos_phi;
    ecef_enu_[8] = sin_phi;
  }
  else
  {
    access::getLogger()->error("Invalid ENU Reference Point provided!");
    throw std::invalid_argument("Invalid ENU Reference Point provided!");
  }
}

const GeoPoint &CoordinateTransform::getENUReferencePoint() const
{
  return enu_ref_point_;
}

size_t CoordinateTransform::getENURef() const
{
  return enu_ref_;
}

bool CoordinateTransform::isENUValid() const
{
  return isValid(enu_ref_point_, false);
}

////////////////////////////////
// ENU Reference Point Management

ENUPoint CoordinateTransform::Geo2ENU(const GeoPoint &pt) const
{
  if (isENUValid())
  {
    if (isValid(pt))
    {
      double dphi = toRadians(pt.latitude) - enu_phi_;
      double dlam = toRadians(pt.longitude) - enu_lam_;
      double dh = static_cast<double>(pt.altitude) - enu_h_;
      double dlam_2 = dlam * dlam;
      double dphi_2 = dphi * dphi;
      double de = (A / enu_tmp1_ + enu_h_) * enu_cp_ * dlam
        - (A * (1 - E2) / enu_tmp1_3_ + enu_h_) * enu_sp_ * dphi * dlam + enu_cp_ * dlam * dh;
      double dn = (A * (1 - E2) / enu_tmp1_3_ + enu_h_) * dphi + 1.5 * enu_cp_ * enu_sp_ * A * E2 * dphi_2
        + enu_sp_2_ * dh * dphi + 0.5 * enu_sp_ * enu_cp_ * (A / enu_tmp1_ + enu_h_) * dlam_2;
      double du = dh - 0.5 * (A - 1.5 * A * E2 * enu_cp_2_ + 0.5 * A * E2 + enu_h_) * dphi_2
        - 0.5 * enu_cp_2_ * (A / enu_tmp1_ - enu_h_) * dlam_2;
      return createENUPoint(de, dn, du);
    }
    else
    {
      access::getLogger()->error("Cannot convert from Geo to ENU: Input Point invalid.");
      throw std::invalid_argument("Cannot convert from Geo to ENU: Input Point invalid.");
    }
  }
  else
  {
    access::getLogger()->error("Cannot convert from Geo to ENU: Reference Point not defined.");
    throw std::invalid_argument("Cannot convert from Geo to ENU: Reference Point not defined.");
  }
}

GeoPoint CoordinateTransform::ENU2Geo(const ENUPoint &pt) const
{
  if (isENUValid())
  {
    if (isValid(pt))
    {
      //* \todo Direct conversion ENU2Geo!
      ECEFPoint ecef = ENU2ECEF(pt);
      return ECEF2Geo(ecef);
    }
    else
    {
      access::getLogger()->error("Cannot convert from ENU to Geo: Input Point invalid.");
      throw std::invalid_argument("Cannot convert from ENU to Geo: Input Point invalid.");
    }
  }
  else
  {
    access::getLogger()->error("Cannot convert from ENU to Geo: Reference Point not defined.");
    throw std::invalid_argument("Cannot convert from ENU to Geo: Reference Point not defined.");
  }
}

///////////////////////
// ECEF/Geo Conversions

ECEFPoint CoordinateTransform::Geo2ECEF(const GeoPoint &pt) const
{
  if (isValid(pt))
  {
    double lat = toRadians(pt.latitude);
    double lon = toRadians(pt.longitude);
    double alt = static_cast<double>(pt.altitude);
    double sinlat = std::sin(lat);
    double coslat = std::cos(lat);
    double n = A / std::sqrt(1 - E2 * sinlat * sinlat);
    double x = (n + alt) * coslat * std::cos(lon);
    double y = (n + alt) * coslat * std::sin(lon);
    double z = (n * (1 - E2) + alt) * sinlat;
    return createECEFPoint(x, y, z);
  }
  else
  {
    access::getLogger()->error("Cannot convert from Geo to ECEF: Input point invalid.");
    throw std::invalid_argument("Cannot convert from Geo to ECEF: Input point invalid.");
  }
}

GeoPoint CoordinateTransform::ECEF2Geo(const ECEFPoint &pt) const
{
  if (isValid(pt))
  {
    double x = static_cast<double>(pt.x);
    double y = static_cast<double>(pt.y);
    double z = static_cast<double>(pt.z);
    double zp = std::abs(z);
    double w2 = x * x + y * y;
    double w = std::sqrt(w2);
    double r2 = w2 + z * z;
    double r = std::sqrt(r2);
    double lon = std::atan2(y, x);
    double s2 = z * z / r2;
    double c2 = w2 / r2;
    double u = A2 / r;
    double v = A3 - A4 / r;
    double lat = 0;
    double c = 0;
    double ss = 0;
    double s = 0;
    if (c2 > 0.3)
    {
      s = (zp / r) * (1.0 + c2 * (A1 + u + s2 * v) / r);
      lat = std::asin(s);
      ss = s * s;
      c = std::sqrt(1.0 - ss);
    }
    else
    {
      c = (w / r) * (1.0 - s2 * (A5 - u - c2 * v) / r);
      lat = std::acos(c);
      ss = 1.0 - c * c;
      s = std::sqrt(ss);
    }
    double g = 1.0 - E2 * ss;
    double rg = A / std::sqrt(g);
    double rf = A6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    double f = c * u + s * v;
    double m = c * v - s * u;
    double p = m / (rf / g + f);
    lat = lat + p;
    double alt = f + m * p / 2.0;
    if (z < 0.0)
    {
      lat *= -1.0;
    }
    return createGeoPoint(Longitude(radians2degree(lon)), Latitude(radians2degree(lat)), Altitude(alt));
  }
  else
  {
    access::getLogger()->error("Cannot convert from ECEF to Geo: Input point invalid.");
    throw std::invalid_argument("Cannot convert from ECEF to Geo: Input point invalid.");
  }
}

///////////////////////
// ECEF/ENU Conversions

ECEFPoint CoordinateTransform::ENU2ECEF(const ENUPoint &pt) const
{
  if (isENUValid())
  {
    if (isValid(pt))
    {
      double x = static_cast<double>(pt.x);
      double y = static_cast<double>(pt.y);
      double z = static_cast<double>(pt.z);
      double ecef_x = ecef_enu_[0] * x + ecef_enu_[3] * y + ecef_enu_[6] * z;
      double ecef_y = ecef_enu_[1] * x + ecef_enu_[4] * y + ecef_enu_[7] * z;
      double ecef_z = ecef_enu_[5] * y + ecef_enu_[8] * z;
      auto const ecef_base = createECEFPoint(ecef_x, ecef_y, ecef_z);
      auto const ecef_result = ecef_base + ecef_ref_point_;
      return ecef_result;
    }
    else
    {
      access::getLogger()->error("Cannot convert from ENU to ECEF: ENU Reference Point invalid.");
      throw std::invalid_argument("Cannot convert from ENU to ECEF: ENU Reference Point invalid.");
    }
  }
  else
  {
    access::getLogger()->error("Cannot convert from ENU to ECEF: Input Point invalid.");
    throw std::invalid_argument("Cannot convert from ENU to ECEF: Input Point invalid.");
  }
}

ENUPoint CoordinateTransform::ECEF2ENU(const ECEFPoint &pt) const
{
  if (isENUValid())
  {
    if (isValid(pt))
    {
      auto const ecef_d = pt - ecef_ref_point_;
      double x = static_cast<double>(ecef_d.x);
      double y = static_cast<double>(ecef_d.y);
      double z = static_cast<double>(ecef_d.z);
      double enu_x = ecef_enu_[0] * x + ecef_enu_[1] * y;
      double enu_y = ecef_enu_[3] * x + ecef_enu_[4] * y + ecef_enu_[5] * z;
      double enu_z = ecef_enu_[6] * x + ecef_enu_[7] * y + ecef_enu_[8] * z;
      return createENUPoint(enu_x, enu_y, enu_z);
    }
    else
    {
      access::getLogger()->error("Cannot convert from ECEF to ENU: ENU Reference Point invalid.");
      throw std::invalid_argument("Cannot convert from ECEF to ENU: ENU Reference Point invalid.");
    }
  }
  else
  {
    access::getLogger()->error("Cannot convert from ECEF to ENU: Input Point invalid.");
    throw std::invalid_argument("Cannot convert from ECEF to ENU: Input Point invalid.");
  }
}

/////////////////
// Useful methods

physics::Distance CoordinateTransform::WGS84_R(Latitude const &lat)
{
  double lat_rad = toRadians(lat);
  double cos_lat = std::cos(lat_rad);
  double sin_lat = std::sin(lat_rad);
  double t1 = A * A * cos_lat;
  double t2 = B * B * sin_lat;
  double t3 = A * cos_lat;
  double t4 = B * sin_lat;
  double d = std::sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
  return physics::Distance(d);
}

double CoordinateTransform::geocentricLatitude(Latitude const &lat)
{
  double lat_rad = toRadians(lat);
  double gclat = std::atan((1.0 - E2) * std::tan(lat_rad));
  return gclat;
}

//////////////
// Static data

size_t CoordinateTransform::instance_counter_ = 0;

} // namespace point
} // namespace map
} // namespace ad
