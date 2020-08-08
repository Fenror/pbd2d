#include "pbd_system.hpp"
#include "constraints.hpp"
#include "point_cloud.hpp"

namespace pbd
{

void PbdSystem::AddLengthConstraint(
    int idx1,
    int idx2,
    double target_len,
    double stiffness,
    int num_iter)
{
  stiffness = glm::clamp(stiffness, 0.0, 1.0);
  length_constraints_.push_back(
      {idx1, idx2, target_len, stiffness, num_iter});
}

void PbdSystem::AddBendConstraint(
    int idx1,
    int idx2,
    int idx3,
    double segment_length,
    double stiffness,
    int num_iter)
{
  stiffness = glm::clamp(stiffness, 0.0, 1.0);
  bend_constraints_.push_back(
      {idx1, idx2, idx3, segment_length, stiffness, num_iter});
}

void PbdSystem::Integrate(double dt)
{
  PointCloud::Integrate(dt);
  HandleLengthConstraints(dt);
  HandleBendConstraints(dt);
}

void PbdSystem::DampVelocity(double damping)
{
  const auto momentum = GetMomentum();
  for (int i = 0; i < GetNumPoints(); ++i)
  {
    AddVelocity(i, damping*(momentum - GetVelocity(i)));
  }
}

void PbdSystem::HandleLengthConstraints(double dt)
{
  for (const auto& c : length_constraints_)
  {
    for (int i = 0; i < c.num_iter; ++i)
    {
      const auto p = GetPoint(c.idx1);
      const auto q = GetPoint(c.idx2);
      const auto pm = GetMass(c.idx1);
      const auto qm = GetMass(c.idx2);
      const auto target_len = c.target_len;
      const auto stiffness = c.stiffness;
      glm::dvec2 dp, dq;
      GetLengthConstraintDelta(
          p,pm,q,qm,target_len,stiffness,&dp,&dq);
      DisplacePointAndUpdateVelocity(c.idx1, dp, dt);
      DisplacePointAndUpdateVelocity(c.idx2, dq, dt);
    }
  }
}

void PbdSystem::HandleBendConstraints(double dt)
{
  for (const auto& c : bend_constraints_)
  {
    for (int i = 0; i < c.num_iter; ++i)
    {
      const auto p = GetPoint(c.idx1);
      const auto q = GetPoint(c.idx2);
      const auto r = GetPoint(c.idx3);
      const auto pm = GetMass(c.idx1);
      const auto qm = GetMass(c.idx2);
      const auto rm = GetMass(c.idx3);
      const auto segment_length = c.segment_length;
      const auto stiffness = c.stiffness;
      glm::dvec2 dp, dq, dr;
      GetBendConstraintDelta(
          p,pm,q,qm,r,rm,
          segment_length,stiffness,
          &dp,&dq,&dr);
      DisplacePointAndUpdateVelocity(c.idx1, dp, dt);
      DisplacePointAndUpdateVelocity(c.idx2, dq, dt);
      DisplacePointAndUpdateVelocity(c.idx3, dr, dt);
    }
  }
}

}
