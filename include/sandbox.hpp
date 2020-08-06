#ifndef SANDBOX_H_
#define SANDBOX_H_

#include "pbd_system.hpp"

#include <memory>
#include <vector>
#include <list>
#include <stdint.h>

#include <glm/glm.hpp>
#include <SDL2/SDL.h>

//namespace pbd
//{
//  class PbdSystem;
//}

class Camera;

namespace sandbox {

enum class Direction{Up, Down, Left, Right};

class Sandbox
{
public:
  Sandbox();
  ~Sandbox();
  void UpdateDynamics(double dt);
  void Quit();
  bool IsRunning() const;

  //Camera control
  Camera* GetCamera() const { return camera_.get(); }
  void Pan(Direction d, bool on_or_off);
  glm::dvec2 GetPanDirection();

  //Geometry control
  const auto& GetPbds() const { return pbds_; }
  double GetPointRadius() const { return point_radius_; }

private:
  std::unique_ptr<Camera> camera_;
  std::vector<std::unique_ptr<pbd::PbdSystem>> pbds_;

  double time_accumulator_ = 0.0;
  double point_radius_ = 0.005;
  double physics_dt_ = 0.01;
  bool running_ = true;

  //Camera
  bool pan_left_ = false;
  bool pan_right_ = false;
  bool pan_up_ = false;
  bool pan_down_ = false;
};

}

#endif
