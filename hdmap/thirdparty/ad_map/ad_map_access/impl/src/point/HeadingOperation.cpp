// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/point/HeadingOperation.hpp"

#include <cmath>

#include "ad/map/access/Operation.hpp"
#include "ad/map/point/ECEFOperation.hpp"
#include "ad/map/point/ENUOperation.hpp"
#include "ad/map/point/Transform.hpp"
#include "ad/physics/AngleOperation.hpp"

namespace ad {
namespace map {
namespace point {

ECEFHeading createECEFHeading(ECEFPoint const &start, ECEFPoint const &end)
{
  ECEFPoint const heading = vectorNorm(end - start);
  ECEFHeading result;
  result.x = heading.x;
  result.y = heading.y;
  result.z = heading.z;
  return result;
}

ECEFHeading createECEFHeading(ENUHeading const &yaw, GeoPoint const &enuReferencePoint)
{
  ECEFPoint const start = toECEF(enuReferencePoint);
  ENUPoint const enuEndPoint = getDirectionalVectorZPlane(yaw);
  ECEFPoint const end = toECEF(enuEndPoint, enuReferencePoint);
  return createECEFHeading(start, end);
}

ENUHeading normalizeENUHeading(ENUHeading const &heading)
{
  return createENUHeading(static_cast<double>(heading));
}

ENUHeading createENUHeading(physics::Angle const &angle)
{
  return ENUHeading(static_cast<double>(physics::normalizeAngleSigned(angle)));
}

ENUHeading createENUHeading(double yawAngleRadian)
{
  return createENUHeading(physics::Angle(yawAngleRadian));
}

ENUHeading createENUHeading(ECEFHeading const &ecefHeading)
{
  return createENUHeading(ecefHeading, access::getENUReferencePoint());
}

ENUHeading createENUHeading(ECEFHeading const &ecefHeading, ECEFPoint const &enuReferencePoint)
{
  return createENUHeading(ecefHeading, toGeo(enuReferencePoint));
}

ENUHeading createENUHeading(ECEFHeading const &ecefHeading, GeoPoint const &enuReferencePoint)
{
  ECEFPoint const enuZero = toECEF(enuReferencePoint);
  ECEFPoint const enuEastAxis = toECEF(getEnuEastAxis(), enuReferencePoint);
  ECEFPoint const enuUpAxis = toECEF(getEnuUpAxis(), enuReferencePoint);

  ECEFHeading const toEast = createECEFHeading(enuZero, enuEastAxis);
  ECEFHeading const toUp = createECEFHeading(enuZero, enuUpAxis);

  double const cosHeading = vectorDotProduct(toEast, ecefHeading);

  ECEFHeading const crossProductVector(vectorCrossProduct(toEast, ecefHeading));
  double const sinHeadingAbs = static_cast<double>(vectorLength(crossProductVector));

  // determine the sign of sine
  ENUHeading result;
  if (std::signbit(vectorDotProduct(crossProductVector, toUp)))
  {
    result = ENUHeading(std::atan2(-sinHeadingAbs, cosHeading));
  }
  else
  {
    result = ENUHeading(std::atan2(sinHeadingAbs, cosHeading));
  }
  // atan2 seems to allow -M_PI, so normalize that one
  if (result == ENUHeading(-M_PI))
  {
    result = ENUHeading(M_PI);
  }
  return result;
}

ENUHeading createENUHeading(ENUPoint const &start, ENUPoint const &end)
{
  ENUPoint const direction = end - start;
  ENUHeading result{std::atan2(static_cast<double>(direction.y), static_cast<double>(direction.x))};
  // atan2 seems to allow -M_PI, so normalize that one
  if (result == ENUHeading(-M_PI))
  {
    result = ENUHeading(M_PI);
  }
  return result;
}

} // namespace point
} // namespace map
} // namespace ad
