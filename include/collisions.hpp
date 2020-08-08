#ifndef COLLISIONS_H_
#define COLLISIONS_H_

#include <glm/glm.hpp>

#include <vector>

class PointCloud;

namespace pbd
{
namespace collisions
{

struct HalfPlane
{
  glm::dvec2 normal;
  glm::dvec2 center;
  double friction_coefficient;
};

void ResolveHalfPlaneCollisions(PointCloud* pc, HalfPlane hp, double dt);

class Collisions
{
public:
  void AddPointCloud(PointCloud* pc);
  void AddHalfPlane(
      glm::dvec2 normal, glm::dvec2 center, double friction_coefficient);
  void ResolveCollisions(double dt);

private:
  void ResolveAllHalfPlaneCollisions(double dt);
  std::vector<HalfPlane> half_planes_;
  std::vector<PointCloud*> point_clouds_;
};


}
}

#endif
