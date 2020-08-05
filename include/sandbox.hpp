#ifndef SANDBOX_H_
#define SANDBOX_H_

#include <memory>
#include <vector>
#include <list>
#include <stdint.h>

#include <glm/glm.hpp>
#include <SDL2/SDL.h>


class Camera;
class Circle;

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
  Circle* GetCircle() const { return circle_.get(); }

private:
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<Circle> circle_;

  double time_accumulator_ = 0.0;
  bool running_ = true;

  //Camera
  bool dragging_camera_ = false;
  bool pan_left_ = false;
  bool pan_right_ = false;
  bool pan_up_ = false;
  bool pan_down_ = false;
  glm::dvec2 panning_direction_{0.0, 0.0};
};


//INPUT
void EventHandler(Sandbox* s, SDL_Event e);

}

#endif
