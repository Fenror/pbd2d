#ifndef DYNAMICAL_SYSTEM_H_
#define DYNAMICAL_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>

class DynamicalSystem
{
public:
  DynamicalSystem(int num_points);
  void Integrate(double dt);
  void DisplacePoint(int i, glm::dvec2 d);
  void AddVelocity(int i, glm::dvec2 v);
  void AddForce(int i, glm::dvec2 F);
  void SpawnNewPoints(std::vector<glm::dvec2> v);
  void RemoveAllPoints();

  //setters & getters
  int GetNumPoints() const;
  std::vector<glm::dvec2> GetPoints() const;
  glm::dvec2 GetPoint(int i) const;
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
  std::vector<glm::dvec2> velocities_;
  std::vector<glm::dvec2> forces_;
  std::vector<double> masses_;
};

#endif