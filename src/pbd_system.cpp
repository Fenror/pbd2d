#include "pbd_system.hpp"
#include "constraints.hpp"
#include "point_cloud.hpp"

namespace pbd
{

PbdSystem::PbdSystem(int num_vertices)
{
  ds_ = std::make_unique<PointCloud>(num_vertices);
}

void PbdSystem::AddLengthConstraint(
    int idx1,
    int idx2,
    double target_len,
    int power)
{
  length_constraints_.push_back({idx1, idx2, target_len, power});
}

void PbdSystem::Integrate(double dt)
{
  ds_->Integrate(dt);
  for (const auto& c : length_constraints_)
  {
    for (int i = 0; i < c.power; ++i)
    {
      const auto p = ds_->GetPoint(c.idx1);
      const auto q = ds_->GetPoint(c.idx2);
      const auto pm = ds_->GetMass(c.idx1);
      const auto qm = ds_->GetMass(c.idx2);
      const auto target_len = c.target_len;
      glm::dvec2 dp, dq;
      GetLengthConstraintDelta(p,pm,q,qm,target_len,&dp,&dq);
      ds_->DisplacePoint(c.idx1, dp);
      ds_->AddVelocity(c.idx1, dp/dt);
      ds_->DisplacePoint(c.idx2, dq);
      ds_->AddVelocity(c.idx2, dq/dt);
    }
  }
}

}
