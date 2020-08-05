#include "constraints.hpp"

#include <glm/gtx/vector_angle.hpp>
#define GLM_FORCE_RADIANS

#include <algorithm>

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

void GetAngleConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    glm::dvec2 r, double rmass,
    double target_angle,
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq,
    glm::dvec2* dr)
{
  const auto edge_l = glm::normalize(q-p);
  const auto edgelen_l = glm::length(q-p);
  const auto edge_r = glm::normalize(r-q);
  const auto edgelen_r = glm::length(r-q);
  const double angle = glm::angle(edge_l, edge_r);

  const double dotprod = glm::dot(edge_l,edge_r);
  const auto l1 = (edge_r - dotprod*edge_l)/edgelen_l;
  const auto r1 = (edge_l - dotprod*edge_r)/edgelen_r;
  const auto l2 = -glm::sqrt(std::max(1e-10,1-dotprod*dotprod))*l1;
  const auto r2 = -glm::sqrt(std::max(1e-10,1-dotprod*dotprod))*r1;
  const double pw = 1/pmass;
  const double qw = 1/qmass;
  const double rw = 1/rmass;
  const double s =
    pw*glm::length2(l1) + qw*glm::length2(l1-r1) + rw*glm::length2(r1);

  *dp = stiffness*0.5*pw*angle*l2/s;
  *dq = stiffness*0.5*qw*angle*(r2-l2)/s;
  *dr = -stiffness*0.5*rw*angle*r2/s;
}

}
