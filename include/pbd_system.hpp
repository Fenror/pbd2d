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
  PointCloud* GetPointCloud() const { return ds_.get(); }

private:
  struct LengthConstraint
  {
    int idx1;
    int idx2;
    double target_len;
    int power;
  };

  std::unique_ptr<PointCloud> ds_;
  std::vector<LengthConstraint> length_constraints_;
};

}
#endif
