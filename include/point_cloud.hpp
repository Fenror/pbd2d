#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <vector>
#include <glm/glm.hpp>

class PointCloud
{
public:
  PointCloud(int num_points);
  ~PointCloud();
  virtual void Integrate(double dt);
  void DisplacePoint(int i, glm::dvec2 d);
  void DisplacePointAndUpdateVelocity(int i, glm::dvec2 d, double dt);
  void AddVelocity(int i, glm::dvec2 v);
  void AddForce(int i, glm::dvec2 F);
  void SetGravity(glm::dvec2 g);
  void SpawnNewPoints(std::vector<glm::dvec2> v);
  void RemoveAllPoints();
  glm::dvec2 GetMomentum() const;

  //setters & getters
  int GetNumPoints() const;
  std::vector<glm::dvec2> GetPoints() const;
  glm::dvec2 GetPoint(int i) const;
  glm::dvec2 GetPointFromPreviousTimestep(int i) const;
  void SetPoint(int i, glm::dvec2 p);
  glm::dvec2 GetVelocity(int i) const;
  void SetVelocity(int i, glm::dvec2 v);
  void SetMass(int i, double m);
  double GetMass(int i) const;
  void SetForce(int i, glm::dvec2 F);
  glm::dvec2 GetCenterOfMass() const;

private:
  int num_points_;
  std::vector<glm::dvec2> points_;
  std::vector<glm::dvec2> points_from_prev_timestep_;
  std::vector<glm::dvec2> velocities_;
  std::vector<glm::dvec2> forces_;
  std::vector<double> masses_;
  glm::dvec2 gravity_{0.0,0.0};
};

#endif
