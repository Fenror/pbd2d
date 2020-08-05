#ifndef ROD_H_
#define ROD_H_

#include <memory>
#include <vector>
#include <list>
#include <glm/glm.hpp>

class DynamicalSystem;

class Rod
{
public:
  Rod(double rod_length, int num_edges);

  void Integrate(double dt);
  void SpawnBetweenPoints(
      glm::dvec2 p1, glm::dvec2 p2, int num_edges);
  void Destroy();
  void DampVelocities(double damping);

  DynamicalSystem* GetDynamicalSystem() const;
  int GetNumEdges() const;
  double GetLength() const;
  void SetLength(double l);
  double GetStretch(unsigned int i) const;
  void SetStretchResistance(int r);
  void SetBendResistance(double r); //Between 0 and 1

private:
  double GetEdgeLength() const { return rod_length_/num_edges_; }
  double GetEdgeLength(int i) const { return glm::length(GetEdge(i)); }
  glm::dvec2 GetEdge(int i) const;
  void EnforceConstraints(double dt);
  void EnforceLengthConstraints();
  void EnforceBendConstraints();
  glm::dvec2 GetGlobalMomentum() const;

  std::unique_ptr<DynamicalSystem> ds_;
  double rod_length_;
  int num_edges_;
  int length_iterations_;
  int bend_iterations_;
  double bend_stiffness_;
};

#endif
