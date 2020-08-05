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

namespace sandbox
{

Sandbox::Sandbox()
{
  camera_ = std::make_unique<Camera>();
  circle_ = std::make_unique<Circle>(glm::dvec2{0.0,0.0}, 1.0);
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
  time_accumulator_ += dt;

  camera_->Displace(GetPanDirection()*dt);
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
