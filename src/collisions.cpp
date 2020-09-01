#include "collisions.hpp"

#include "point_cloud.hpp"
#include "geometry.hpp"

#include <glm/glm.hpp>

namespace pbd
{
namespace collisions
{

void Collisions::AddPointCloud(PointCloud* pc)
{
  point_clouds_.push_back(pc);
}

void Collisions::AddRod(PointCloud *pc)
{
  for (int i = 0; i < pc->GetNumPoints()-1; ++i)
  {
    const double friction_coefficient = 0.0;
    AddLineSeg(pc, i, i+1, friction_coefficient);
  }
  for (int i = 0; i < pc->GetNumPoints(); ++i)
  {
    AddPoint(pc, i);
  }
}

void Collisions::AddHalfPlane(
    glm::dvec2 normal, glm::dvec2 center, double friction_coefficient)
{
  friction_coefficient = glm::clamp(friction_coefficient, 0.0, 1.0);
  half_planes_.push_back({normal, center, friction_coefficient});
}

void Collisions::AddLineSeg(PointCloud* pc, int idx1, int idx2,
    double friction_coefficient)
{
  line_segs_.push_back({pc, idx1, idx2, friction_coefficient});
}

void Collisions::AddPoint(PointCloud* pc, int idx)
{
  points_.push_back({pc, idx});
}

void Collisions::ResolveCollisions(double dt)
{
  ResolveAllPointLineSegCollisions(dt);
  ResolveAllHalfPlaneCollisions(dt);
  ResolveAllPolygonPointCollisions(dt);
}

void Collisions::ResolveAllHalfPlaneCollisions(double dt)
{
  for (const auto& hp : half_planes_)
  {
    for (const auto& pc : point_clouds_)
    {
      ResolveHalfPlaneCollisions(pc, hp, dt);
    }
  }
}

void Collisions::ResolveAllPointLineSegCollisions(double dt)
{
  for (const auto& pt : points_)
  {
    for (const auto& l : line_segs_)
    {
      if (pt.pc == l.pc)
        continue;

      const auto p = pt.pc->GetPoint(pt.idx);
      const auto q1 = l.pc->GetPoint(l.idx1);
      const auto q2 = l.pc->GetPoint(l.idx2);
      const auto d = geometry::PointLinesegDistance(p,q1,q2);
      const auto r = pt.pc->GetRadius(pt.idx);

      if (glm::abs(d-r) < 0.01*r)
        continue;

      if (d < r)
      {
        const auto pmass = pt.pc->GetMass(pt.idx);
        const auto q1mass = l.pc->GetMass(l.idx1);
        const auto q2mass = l.pc->GetMass(l.idx2);
        const glm::dvec2 tangent = glm::normalize(q2-q1);
        const glm::dvec2 proj = glm::dot(p-q1,tangent)*tangent;
        const glm::dvec2 dir = glm::normalize(p-q1-proj);
        const double total_mass = pmass+q1mass+q2mass;
        const double pw = pmass/total_mass;
        const double qw = (q1mass+q2mass)/total_mass;
        const double len = r-d;
        const auto dp = pw*len*dir;
        const auto dq1 = -qw*len*dir;
        const auto dq2 = -qw*len*dir;
        pt.pc->DisplacePointAndUpdateVelocity(pt.idx, dp, dt);
        l.pc->DisplacePointAndUpdateVelocity(l.idx1, dq1, dt);
        l.pc->DisplacePointAndUpdateVelocity(l.idx2, dq2, dt);

        const glm::dvec2 new_vel{
          l.friction_coefficient, l.friction_coefficient};
        pt.pc->SetVelocity(pt.idx, new_vel);
        l.pc->SetVelocity(l.idx1, new_vel);
        l.pc->SetVelocity(l.idx2, new_vel);
        continue;
      }

      const auto p_old = pt.pc->GetPointFromPreviousTimestep(pt.idx);
      glm::dvec2 intersec;
      const auto tunneled = geometry::LinesegLinesegIntersection(
          p_old, p, q1, q2, &intersec);

      if (tunneled)
      {
        pt.pc->DisplacePointAndUpdateVelocity(
            pt.idx, 2.0*(intersec-p), dt);
      }
    }
  }
}

void Collisions::ResolveAllPolygonPointCollisions(double dt)
{
  for (const auto& poly : polygons_)
  {
    for (const auto& point : points_)
    {
      if (poly.pc == point.pc)
      {
        continue;
      }
      ResolvePolygonPointCollision(poly, point, dt);
    }
  }
}

void Collisions::AddPolygon(PointCloud *pc)
{
  std::vector<LineSeg> sides;
  const auto num_points = pc->GetNumPoints();
  for (int i = 0; i < num_points; ++i)
  {
    sides.push_back({pc, i, (i+1)%num_points, 0.0});
  }
  polygons_.push_back({pc, sides});
}

bool DetectContinuousPointLineSegCollision(
    glm::dvec2 p, glm::dvec2 p_prev,
    glm::dvec2 q1, glm::dvec2 q2)
{
  glm::dvec2 intersec;
  return geometry::LinesegLinesegIntersection(
      p_prev, p, q1, q2, &intersec);
}

void ResolveHalfPlaneCollisions(PointCloud* pc, HalfPlane hp, double dt)
{
  for (int i = 0; i < pc->GetNumPoints(); ++i)
  {
    const auto p = pc->GetPoint(i);
    const auto d = glm::dot(p - hp.center, hp.normal);
    if (d < 0.0)
    {
      pc->DisplacePointAndUpdateVelocity(i, -d*hp.normal, dt);
      const auto v = pc->GetVelocity(i);
      const auto vn = glm::dot(v,hp.normal)*hp.normal;
      const auto vt = v-vn;
      pc->SetVelocity(i, vn+hp.friction_coefficient*vt);
    }
  }
}

void ResolvePointLineSegCollision(
    glm::dvec2 p, double pmass,
    glm::dvec2 q1, double q1mass,
    glm::dvec2 q2, double q2mass,
    glm::dvec2* dp, glm::dvec2* dq1, glm::dvec2* dq2)
{
  glm::dvec2 closest_point;

  const glm::dvec2 tangent = glm::normalize(q2-q1);
  const glm::dvec2 proj = glm::dot(p-q1,tangent)*tangent;
  const glm::dvec2 dir = proj-(p-q1);
  const double total_mass = pmass+q1mass+q2mass;
  const double pw = pmass/total_mass;
  const double qw = q1mass+q2mass/total_mass;
  *dp = pw*dir;
  *dq1 = -qw*dir;
  *dq2 = -qw*dir;
}

void ResolvePolygonPointCollision(
    Polygon poly,
    Point point,
    double dt)
{
  const glm::dvec2 ray_dir{1.0, 0.0};

  int num_intersections = 0;
  const auto p = point.pc->GetPoint(point.idx);
  for (const auto& l : poly.line_segments)
  {
    glm::dvec2 intersec;
    const auto q1 = l.pc->GetPoint(l.idx1);
    const auto q2 = l.pc->GetPoint(l.idx2);
    const auto hit = geometry::RayLinesegIntersection(
        p, ray_dir, q1, q2, &intersec);

    if (hit)
    {
      num_intersections += 1;
    }
  }

  if (num_intersections % 2 == 1)
  {
    const auto& l0 = poly.line_segments[0];
    const auto q1 = l0.pc->GetPoint(l0.idx1);
    const auto q2 = l0.pc->GetPoint(l0.idx2);
    const auto smallest_dist = geometry::PointLinesegDistance(p, q1, q2);
    glm::dvec2 cp;
    int closest_idx = 0;

    for (int i = 0; i < poly.line_segments.size(); ++i)
    {
      glm::dvec2 intersec;
      const auto& l = poly.line_segments[i];
      const auto q1 = l.pc->GetPoint(l.idx1);
      const auto q2 = l.pc->GetPoint(l.idx2);
      const auto d = geometry::PointLinesegDistance(
          p, q1, q2, &intersec);

      if (d <= smallest_dist)
      {
        closest_idx = i;
        cp = intersec;
      }
    }

    const auto& cl = poly.line_segments[closest_idx];
    const auto dir = cp - p;
    const auto pmass = point.pc->GetMass(point.idx);
    const auto q1mass = cl.pc->GetMass(cl.idx1);
    const auto q2mass = cl.pc->GetMass(cl.idx2);
    const double total_mass = pmass+q1mass+q2mass;
    const double pw = pmass/total_mass;
    const double qw = q1mass+q2mass/total_mass;
    const auto dp = pw*dir;
    const auto dq1 = -qw*dir;
    const auto dq2 = -qw*dir;
    point.pc->DisplacePointAndUpdateVelocity(point.idx, dp, dt);
    cl.pc->DisplacePointAndUpdateVelocity(cl.idx1, dq1, dt);
    cl.pc->DisplacePointAndUpdateVelocity(cl.idx2, dq2, dt);
  }

}

}
}
