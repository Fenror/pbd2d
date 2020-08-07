#include "pbd_system.hpp"
#include "constraints.hpp"
#include "point_cloud.hpp"

namespace pbd
{

void PbdSystem::AddLengthConstraint(
    int idx1,
    int idx2,
    double target_len,
    int power)
{
  length_constraints_.push_back({idx1, idx2, target_len, power});
}

void PbdSystem::AddAngleConstraint(
    int idx1,
    int idx2,
    int idx3,
    double target_angle,
    double stiffness,
    int power)
{
  angle_constraints_.push_back(
      {idx1, idx2, idx3, target_angle, stiffness, power});
}

void PbdSystem::Integrate(double dt)
{
  PointCloud::Integrate(dt);
  HandleLengthConstraints(dt);
  HandleAngleConstraints(dt);
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
    for (int i = 0; i < c.power; ++i)
    {
      const auto p = GetPoint(c.idx1);
      const auto q = GetPoint(c.idx2);
      const auto pm = GetMass(c.idx1);
      const auto qm = GetMass(c.idx2);
      const auto target_len = c.target_len;
      glm::dvec2 dp, dq;
      GetLengthConstraintDelta(p,pm,q,qm,target_len,&dp,&dq);
      DisplacePointAndUpdateVelocity(c.idx1, dp, dt);
      DisplacePointAndUpdateVelocity(c.idx2, dq, dt);
    }
  }
}

void PbdSystem::HandleAngleConstraints(double dt)
{
  for (const auto& c : angle_constraints_)
  {
    for (int i = 0; i < c.power; ++i)
    {
      const auto p = GetPoint(c.idx1);
      const auto q = GetPoint(c.idx2);
      const auto r = GetPoint(c.idx3);
      const auto pm = GetMass(c.idx1);
      const auto qm = GetMass(c.idx2);
      const auto rm = GetMass(c.idx3);
      const auto target_angle = c.target_angle;
      const auto stiffness = c.stiffness;
      glm::dvec2 dp, dq, dr;
      GetAngleConstraintDelta(
          p,pm,q,qm,r,rm,
          target_angle,stiffness,
          &dp,&dq,&dr);
      DisplacePointAndUpdateVelocity(c.idx1, dp, dt);
      DisplacePointAndUpdateVelocity(c.idx2, dq, dt);
      DisplacePointAndUpdateVelocity(c.idx3, dr, dt);
    }
  }
}

}
