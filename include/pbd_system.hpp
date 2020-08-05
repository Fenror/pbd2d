#ifndef PBD_SYSTEM_H_
#define PBD_SYSTEM_H_

#include <glm/glm.hpp>
#include <memory>
#include <vector>

class PointCloud;

namespace pbd
{

class PbdSystem
{
public:
  PbdSystem(int num_vertices);
  void Integrate(double dt);
  void AddLengthConstraint(
      int idx1, int idx2, double target_len, int power);
  void AddAngleConstraint(
      int idx1, int idx2, int idx3,
      double target_angle, double stiffness, int power);
  PointCloud* GetPointCloud() const { return ds_.get(); }

private:
  void HandleLengthConstraints(double dt);
  void HandleAngleConstraints(double dt);

  struct LengthConstraint
  {
    int idx1;
    int idx2;
    double target_len;
    int power;
  };

  struct AngleConstraint
  {
    int idx1;
    int idx2;
    int idx3;
    double target_angle;
    double stiffness;
    int power;
  };

  std::unique_ptr<PointCloud> ds_;
  std::vector<LengthConstraint> length_constraints_;
  std::vector<AngleConstraint> angle_constraints_;
};

}
#endif
