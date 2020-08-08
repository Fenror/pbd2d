#include "collisions.hpp"

#include "point_cloud.hpp"

namespace pbd
{
namespace collisions
{

void Collisions::AddPointCloud(PointCloud* pc)
{
  point_clouds_.push_back(pc);
}

void Collisions::AddHalfPlane(
    glm::dvec2 normal, glm::dvec2 center, double friction_coefficient)
{
  friction_coefficient = glm::clamp(friction_coefficient, 0.0, 1.0);
  half_planes_.push_back({normal, center, friction_coefficient});
}

void Collisions::ResolveCollisions(double dt)
{
  ResolveAllHalfPlaneCollisions(dt);
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

}
}
