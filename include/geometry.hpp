#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <glm/glm.hpp>

namespace geometry
{
bool LinesegLinesegIntersection(
    const glm::dvec2& p1,
    const glm::dvec2& p2,
    const glm::dvec2& q1,
    const glm::dvec2& q2,
    glm::dvec2* intersection_point);

double PointLinesegDistance(
    const glm::dvec2& v,
    const glm::dvec2& e1,
    const glm::dvec2& e2,
    glm::dvec2* closest_point);

double PointLinesegDistance(
    const glm::dvec2& v,
    const glm::dvec2& e1,
    const glm::dvec2& e2);

double PointLineDistance(
    const glm::dvec2& v,
    const glm::dvec2& point_on_line,
    const glm::dvec2& normal);

double PointLineSignedDistance(
    const glm::dvec2& v,
    const glm::dvec2& point_on_line,
    const glm::dvec2& normal);

bool RayLinesegIntersection(
    const glm::dvec2& ray_origin,
    const glm::dvec2& ray_direction,
    const glm::dvec2& p1,
    const glm::dvec2& p2,
    glm::dvec2* intersection_point);

double Cross(
    const glm::dvec2& u,
    const glm::dvec2& v);

struct Rect { double x1, x2, y1, y2; };
}

#endif
