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
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq)
{
  const auto u = q-p;
  const auto ulen = glm::length(u);
  const auto C = ulen - target_len;
  const auto dCdu = u/ulen;
  const auto dCdp = -dCdu;
  const auto dCdq = dCdu;
  const auto pw = 1/pmass;
  const auto qw = 1/qmass;
  const auto s = C/(pw*glm::length2(dCdp) + qw*glm::length2(dCdq));
  *dp = -s*stiffness*pw*dCdp;
  *dq = -s*stiffness*qw*dCdq;
}

void GetBendConstraintDelta(
    glm::dvec2 p, double pmass,
    glm::dvec2 q, double qmass,
    glm::dvec2 r, double rmass,
    double segment_length,
    double stiffness,
    glm::dvec2* dp,
    glm::dvec2* dq,
    glm::dvec2* dr)
{
  const auto u = q-p;
  const auto v = r-q;
  const glm::dvec2 uperp{u.y, -u.x};
  const glm::dvec2 vperp{v.y, -v.x};
  const auto ulen = glm::length(u);
  const auto vlen = glm::length(v);
  const auto ub = glm::normalize(u);
  const auto vb = glm::normalize(v);
  const auto uperpb = glm::normalize(uperp);
  const auto vperpb = glm::normalize(vperp);
  const auto udotv = glm::dot(u,v);
  const auto udotvperp = glm::dot(u,vperp);
  if (glm::abs(ulen*vlen+udotv) < 1e-10)
  {
    *dp = {0.0, 0.0};
    *dq = {0.0, 0.0};
    *dr = {0.0, 0.0};
    return;
  }
  const auto a = 1.0/(ulen*vlen+udotv);
  const auto dbdu = 2.0*a*vlen*(vperpb - udotvperp*a*(ub+vb));
  const auto dbdv = -2.0*a*ulen*(uperpb + udotvperp*a*(ub+vb));
  const auto b = 2.0*udotvperp/(ulen*vlen+udotv)/segment_length;
  const auto dCdu = 2.0*b*dbdu/segment_length;
  const auto dCdv = 2.0*b*dbdv/segment_length;

  const auto dCdp = -dCdu;
  const auto dCdq = dCdu-dCdv;
  const auto dCdr = dCdv;
  const auto C = b*b;
  if (C < 1e-10)
  {
    *dp = {0.0, 0.0};
    *dq = {0.0, 0.0};
    *dr = {0.0, 0.0};
    return;
  }
  const auto pw = 1/pmass;
  const auto qw = 1/qmass;
  const auto rw = 1/rmass;
  const auto s = C/(pw*glm::length2(dCdp) +
                    qw*glm::length2(dCdq) +
                    rw*glm::length2(dCdr));
  *dp = -s*stiffness*pw*dCdp;
  *dq = -s*stiffness*qw*dCdq;
  *dr = -s*stiffness*rw*dCdr;
}

}
