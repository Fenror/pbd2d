#include "geometry.hpp"
#include <algorithm>

namespace geometry
{
bool LinesegLinesegIntersection(
    const glm::dvec2& p1,
    const glm::dvec2& p2,
    const glm::dvec2& q1,
    const glm::dvec2& q2,
    glm::dvec2* intersection_point)
{
  const glm::dvec2 r = p2 - p1;
  const glm::dvec2 s = q2 - q1;

  const double n = Cross(q1-p1,r);
  const double d = Cross(r,s);
  const double u = n/d;
  const double t = Cross(q1-p1,s)/d;

  if (!(d != 0.0 && 0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0))
  {
    return false;
  }
  else
  {
    *intersection_point = q1 + u*s;
    return true;
  }

}

double PointLinesegDistance(
    const glm::dvec2& v,
    const glm::dvec2& e1,
    const glm::dvec2& e2,
    glm::dvec2* closest_point)
{
  const double l2 = glm::dot(e1-e2,e1-e2);
  if (l2 == 0.0) return glm::length(v-e1);
  const double t = std::max(0.0, std::min(1.0, dot(v - e1, e2 - e1) / l2));
  *closest_point = e1 + t * (e2 - e1);
  return glm::length(v-*closest_point);
}

double PointLinesegDistance(
    const glm::dvec2& v,
    const glm::dvec2& e1,
    const glm::dvec2& e2)
{
  const double l2 = glm::dot(e1-e2,e1-e2);
  if (l2 == 0.0) return glm::length(v-e1);
  const double t = std::max(0.0, std::min(1.0, dot(v - e1, e2 - e1) / l2));
  auto closest_point = e1 + t * (e2 - e1);
  return glm::length(v-closest_point);
}

double PointLineDistance(
    const glm::dvec2& v,
    const glm::dvec2& p,
    const glm::dvec2& normal)
{
  return glm::abs(glm::dot(v-p,normal));
}

double PointLineSignedDistance(
    const glm::dvec2& v,
    const glm::dvec2& p,
    const glm::dvec2& normal)
{
  return glm::dot(v-p,normal);
}

double Cross(
    const glm::dvec2& u,
    const glm::dvec2& v)
{
  return u.x*v.y - u.y*v.x;
}
}
