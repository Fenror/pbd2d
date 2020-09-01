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

struct LineSeg
{
  PointCloud* pc;
  int idx1;
  int idx2;
  double friction_coefficient;
};

struct Point
{
  PointCloud* pc;
  int idx;
};

struct Polygon
{
  PointCloud* pc;
  std::vector<LineSeg> line_segments;
};

bool DetectContinuousPointLineSegCollision(
    glm::dvec2 p, glm::dvec2 p_prev,
    glm::dvec2 q1, glm::dvec2 q2);

void ResolveHalfPlaneCollisions(PointCloud* pc, HalfPlane hp, double dt);
void ResolvePointLineSegCollision(
    glm::dvec2 p, double pmass,
    glm::dvec2 q1, double q1mass,
    glm::dvec2 q2, double q2mass,
    glm::dvec2* dp, glm::dvec2* dq1, glm::dvec2* dq2);
void ResolvePolygonPointCollision(
    Polygon poly,
    Point point,
    double dt);

class Collisions
{
public:
  void AddPointCloud(PointCloud* pc);
  void AddPolygon(PointCloud *pc);
  void AddRod(PointCloud *pc);
  void AddLineSeg(PointCloud* pc, int idx1, int idx2,
      double friction_coefficient);
  void AddPoint(PointCloud* pc, int idx);
  void AddHalfPlane(
      glm::dvec2 normal, glm::dvec2 center, double friction_coefficient);
  void ResolveCollisions(double dt);

private:
  void ResolveAllHalfPlaneCollisions(double dt);
  void ResolveAllPointLineSegCollisions(double dt);
  void ResolveAllPolygonPointCollisions(double dt);
  std::vector<HalfPlane> half_planes_;
  std::vector<PointCloud*> point_clouds_;
  std::vector<LineSeg> line_segs_;
  std::vector<Point> points_;
  std::vector<Polygon> polygons_;
};


}
}

#endif
