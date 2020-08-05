#include "sandbox.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <SDL2/SDL.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

#include "camera.hpp"
#include "circle.hpp"
#include "osksdl.hpp"
#include "point_cloud.hpp"
#include "pbd_system.hpp"
#include "pbd_factory.hpp"

namespace sandbox
{

Sandbox::Sandbox()
{
  camera_ = std::make_unique<Camera>();
  pbd_ = pbd::MakeRod(0.5, 10, 0.02);
  pbd_->GetPointCloud()->SetForce(0,{0.1,0.1});
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
    pbd_->Integrate(physics_dt_);
    cur_phys_time += physics_dt_;
  }
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


//INPUT
void EventHandler(Sandbox* s, SDL_Event e)
{
  if (e.key.type == SDL_KEYDOWN)
  {
    switch (e.key.keysym.sym)
    {
      case SDLK_q:
        s->Quit();
        break;
      case SDLK_a:
        s->Pan(Direction::Left, true);
        break;
      case SDLK_d:
        s->Pan(Direction::Right, true);
        break;
      case SDLK_w:
        s->Pan(Direction::Up, true);
        break;
      case SDLK_s:
        s->Pan(Direction::Down, true);
        break;
    }
  }

  if (e.key.type == SDL_KEYUP)
  {
    switch (e.key.keysym.sym)
    {
      case SDLK_q:
        s->Quit();
        break;
      case SDLK_a:
        s->Pan(Direction::Left, false);
        break;
      case SDLK_d:
        s->Pan(Direction::Right, false);
        break;
      case SDLK_w:
        s->Pan(Direction::Up, false);
        break;
      case SDLK_s:
        s->Pan(Direction::Down, false);
        break;
    }
  }
}


}
