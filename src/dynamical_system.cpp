#include "dynamical_system.hpp"

DynamicalSystem::DynamicalSystem(int num_points)
  : num_points_(num_points)
  , points_(num_points)
  , velocities_(num_points)
  , forces_(num_points)
  , masses_(num_points)
{
  for (int i = 0; i < num_points_; ++i)
  {
    points_[i] = {0.0, 0.0};
    velocities_[i] = {0.0, 0.0};
    forces_[i] = {0.0, 0.0};
    masses_[i] = 1.0;
  }
}

void DynamicalSystem::Integrate(double dt)
{
  for (int i = 0; i < num_points_; ++i)
  {
    velocities_[i] += masses_[i]*forces_[i]*dt;
    points_[i] += velocities_[i]*dt;
  }
}

void DynamicalSystem::DisplacePoint(int i, glm::dvec2 d)
{
  points_[i] += d;
}

void DynamicalSystem::AddVelocity(int i, glm::dvec2 v)
{
  velocities_[i] += v;
}

void DynamicalSystem::AddForce(int i, glm::dvec2 F)
{
  forces_[i] += F;
}

void DynamicalSystem::SpawnNewPoints(std::vector<glm::dvec2> v)
{
  points_ = v;
  num_points_ = v.size();

  velocities_.resize(num_points_);
  forces_.resize(num_points_);
  masses_.resize(num_points_);

  for (int i = 0; i < num_points_; ++i)
  {
    velocities_[i] = {0.0, 0.0};
    forces_[i] = {0.0, 0.0};
    masses_[i] = 1.0;
  }
}

void DynamicalSystem::RemoveAllPoints()
{
  num_points_ = 0;
  points_.resize(0);
  velocities_.resize(0);
  forces_.resize(0);
  masses_.resize(0);
}

//setters & getters
int DynamicalSystem::GetNumPoints() const
{
  return num_points_;
}

std::vector<glm::dvec2> DynamicalSystem::GetPoints() const
{
  return points_;
}

glm::dvec2 DynamicalSystem::GetPoint(int i) const
{
  return points_[i];
}

void DynamicalSystem::SetPoint(int i, glm::dvec2 p)
{
  points_[i] = p;
}

glm::dvec2 DynamicalSystem::GetVelocity(int i) const
{
  return velocities_[i];
}

void DynamicalSystem::SetVelocity(int i, glm::dvec2 v)
{
  velocities_[i] = v;
}

void DynamicalSystem::SetMass(int i, double m)
{
  masses_[i] = m;
}

double DynamicalSystem::GetMass(int i) const
{
  return masses_[i];
}

void DynamicalSystem::SetForce(int i, glm::dvec2 F)
{
  forces_[i] = F;
}

glm::dvec2 DynamicalSystem::GetCenterOfMass() const
{
  glm::dvec2 center_of_mass{0.0,0.0};

  for (int i = 0; i < num_points_; ++i)
  {
    center_of_mass += masses_[i]*points_[i];
  }
  center_of_mass /= num_points_;

  return center_of_mass;
}
