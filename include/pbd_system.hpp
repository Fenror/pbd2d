#ifndef PBD_SYSTEM_H_
#define PBD_SYSTEM_H_

#include "point_cloud.hpp"

#include <glm/glm.hpp>
#include <memory>
#include <vector>


namespace pbd
{

class PbdSystem : public PointCloud
{
public:
  using PointCloud::PointCloud;
  void Integrate(double dt) override;
  void DampVelocity(double damping);
  void AddLengthConstraint(
      int idx1, int idx2,
      double target_len, double stiffness, int num_iter);
  void AddBendConstraint(
      int idx1, int idx2, int idx3,
      double target_angle, double stiffness, int num_iter);

private:
  void HandleLengthConstraints(double dt);
  void HandleBendConstraints(double dt);

  struct LengthConstraint
  {
    int idx1;
    int idx2;
    double target_len;
    double stiffness;
    int num_iter;
  };

  struct BendConstraint
  {
    int idx1;
    int idx2;
    int idx3;
    double segment_length;
    double stiffness;
    int num_iter;
  };

  std::vector<LengthConstraint> length_constraints_;
  std::vector<BendConstraint> bend_constraints_;
};

}
#endif
