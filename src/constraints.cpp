#include "constraints.hpp"

namespace pbd
{

void GetLengthConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    double target_len,
    glm::dvec2* dp,
    glm::dvec2* dq)
{
  const auto edge = q-p;
  const auto edge_len = glm::length(edge);
  const double pw = 1/pmass;
  const double qw = 1/qmass;
  *dp = pw/(pw+qw)*0.5*(1.0 - target_len/edge_len)*edge;
  *dq = qw/(pw+qw)*0.5*(target_len/edge_len - 1.0)*edge;
}

}
