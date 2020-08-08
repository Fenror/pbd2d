#include "sandbox.hpp"

#include "camera.hpp"
#include "circle.hpp"
#include "osksdl.hpp"
#include "point_cloud.hpp"
#include "pbd_system.hpp"
#include "pbd_factory.hpp"
#include "collisions.hpp"

#include <SDL2/SDL.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <limits>


namespace sandbox
{

Sandbox::Sandbox()
{
  camera_ = std::make_unique<Camera>();

  //Add rod
  const double rod_len = 0.5;
  const int num_edges = 3;
  const double stretch_resistance = 1.0;
  const double bend_resistance = 1.0;
  pbds_.push_back(pbd::MakeRod(
        rod_len, 0.1, num_edges, stretch_resistance, bend_resistance));
  pbds_[0]->SetGravity({0,-9.82});

  //Add square
  const double stiffness = 0.3;
  pbds_.push_back(pbd::MakeSquare(0.1, stiffness));
  pbds_[1]->SetGravity({0,-9.82});

  //Add floor
  const glm::dvec2 normal{0.0, 1.0};
  const glm::dvec2 center{0.0, 0.0};
  const double friction_coeff = 0.0;
  collisions_.AddHalfPlane(normal, center, friction_coeff);
  collisions_.AddPointCloud(pbds_[0].get());
  collisions_.AddPointCloud(pbds_[1].get());
}

Sandbox::~Sandbox() {}

void Sandbox::Quit()
{
  running_ = false;
}

bool Sandbox::IsRunning() const
{
  return running_;
}

void Sandbox::UpdateDynamics(double dt)
{
  auto cur_phys_time = time_accumulator_;
  time_accumulator_ += dt;

  camera_->Displace(GetPanDirection()*dt);

  while (cur_phys_time < time_accumulator_)
  {
    for (auto& pbd : pbds_)
      pbd->Integrate(physics_dt_);

    collisions_.ResolveCollisions(physics_dt_);

    for (const auto& sel : selections_)
    {
      const int pbd_idx = sel.first.first;
      const int point_idx = sel.first.second;
      const auto p = pbds_[pbd_idx]->GetPoint(point_idx);
      const auto d = attractor_point_ - p;
      pbds_[pbd_idx]->DisplacePointAndUpdateVelocity(
          point_idx, d, physics_dt_);
    }

    cur_phys_time += physics_dt_;
  }

}

glm::dvec2 Sandbox::GetPoint(int pbd_idx, int point_idx) const
{
  return pbds_[pbd_idx]->GetPoint(point_idx);
}

void Sandbox::SelectPoint(int pbd_idx, int point_idx, Selection type)
{
  std::pair<int,int> key{pbd_idx, point_idx};
  if (selections_.count(key))
  {
    selections_.erase(key);
  }
  else
  {
    const double inf = std::numeric_limits<double>::max();
    selections_.emplace(std::pair<int,int>{pbd_idx, point_idx}, type);
  }
}

void Sandbox::DeselectAll()
{
  selections_.clear();
}

void Sandbox::SetAttractorPoint(glm::dvec2 p)
{
  attractor_point_ = p;
}

void Sandbox::Pan(Direction d, bool onoff)
{
  switch (d)
  {
    case Direction::Up:
      pan_up_ = onoff;
      break;
    case Direction::Down:
      pan_down_ = onoff;
      break;
    case Direction::Left:
      pan_left_ = onoff;
      break;
    case Direction::Right:
      pan_right_ = onoff;
      break;
  }
}

glm::dvec2 Sandbox::GetPanDirection()
{
  auto dir = glm::dvec2{0.0, 0.0};

  if (pan_up_)
    dir += glm::dvec2{0.0, 1.0};

  if (pan_down_)
    dir += glm::dvec2{0.0, -1.0};

  if (pan_left_)
    dir += glm::dvec2{-1.0, 0.0};

  if (pan_right_)
    dir += glm::dvec2{1.0, 0.0};

  return dir;
}

}
