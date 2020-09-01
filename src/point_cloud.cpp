#include "point_cloud.hpp"

PointCloud::PointCloud(int num_points)
  : num_points_(num_points)
  , points_(num_points)
  , points_from_prev_timestep_(num_points)
  , velocities_(num_points)
  , forces_(num_points)
  , masses_(num_points)
  , radii_(num_points)
{
  for (int i = 0; i < num_points_; ++i)
  {
    points_[i] = {0.0, 0.0};
    velocities_[i] = {0.0, 0.0};
    forces_[i] = {0.0, 0.0};
    masses_[i] = 1.0;
    radii_[i] = 0.01;
  }

  points_from_prev_timestep_ = points_;
}

void PointCloud::Integrate(double dt)
{
  points_from_prev_timestep_ = points_;
  for (int i = 0; i < num_points_; ++i)
  {
    velocities_[i] += forces_[i]*dt/masses_[i] + gravity_*dt;
    points_[i] += velocities_[i]*dt;
  }
}

void PointCloud::DisplacePoint(int i, glm::dvec2 d)
{
  points_[i] += d;
}

void PointCloud::DisplacePointAndUpdateVelocity(
    int i, glm::dvec2 d, double dt)
{
  points_[i] += d;
  velocities_[i] += d/dt;
}

void PointCloud::DisplaceCloud(glm::dvec2 d)
{
  for (int i = 0; i < num_points_; ++i)
  {
    DisplacePoint(i, d);
  }
}

void PointCloud::AddVelocity(int i, glm::dvec2 v)
{
  velocities_[i] += v;
}

void PointCloud::AddForce(int i, glm::dvec2 F)
{
  forces_[i] += F;
}

void PointCloud::SetGravity(glm::dvec2 g)
{
  gravity_ = g;
}

void PointCloud::SpawnNewPoints(std::vector<glm::dvec2> v)
{
  points_ = v;
  points_from_prev_timestep_ = v;
  num_points_ = v.size();

  velocities_.resize(num_points_);
  forces_.resize(num_points_);
  masses_.resize(num_points_);
  radii_.resize(num_points_);

  for (int i = 0; i < num_points_; ++i)
  {
    velocities_[i] = {0.0, 0.0};
    forces_[i] = {0.0, 0.0};
    masses_[i] = 1.0;
    radii_[i] = 0.01;
  }
}

void PointCloud::RemoveAllPoints()
{
  num_points_ = 0;
  points_.resize(0);
  points_from_prev_timestep_.resize(0);
  velocities_.resize(0);
  forces_.resize(0);
  masses_.resize(0);
  radii_.resize(0);
}

//setters & getters
int PointCloud::GetNumPoints() const
{
  return num_points_;
}

std::vector<glm::dvec2> PointCloud::GetPoints() const
{
  return points_;
}

glm::dvec2 PointCloud::GetPoint(int i) const
{
  return points_[i];
}

glm::dvec2 PointCloud::GetPointFromPreviousTimestep(int i) const
{
  return points_from_prev_timestep_[i];
}

void PointCloud::SetPoint(int i, glm::dvec2 p)
{
  points_[i] = p;
}

glm::dvec2 PointCloud::GetVelocity(int i) const
{
  return velocities_[i];
}

void PointCloud::SetVelocity(int i, glm::dvec2 v)
{
  velocities_[i] = v;
}

void PointCloud::SetMass(int i, double m)
{
  masses_[i] = m;
}

double PointCloud::GetMass(int i) const
{
  return masses_[i];
}

void PointCloud::SetRadii(double r)
{
  std::fill(radii_.begin(), radii_.end(), r);
}

double PointCloud::GetRadius(int i) const
{
  return radii_[i];
}

void PointCloud::SetForce(int i, glm::dvec2 F)
{
  forces_[i] = F;
}

glm::dvec2 PointCloud::GetCenterOfMass() const
{
  glm::dvec2 center_of_mass{0.0,0.0};

  for (int i = 0; i < num_points_; ++i)
  {
    center_of_mass += masses_[i]*points_[i];
  }
  center_of_mass /= num_points_;

  return center_of_mass;
}

glm::dvec2 PointCloud::GetMomentum() const
{
  glm::dvec2 momentum{0.0,0.0};

  for (int i = 0; i < num_points_; ++i)
    momentum += velocities_[i];

  momentum /= num_points_;

  return momentum;
}
