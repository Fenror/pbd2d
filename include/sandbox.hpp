#ifndef SANDBOX_H_
#define SANDBOX_H_

#include "pbd_system.hpp"

#include "collisions.hpp"

#include <list>
#include <map>
#include <memory>
#include <stdint.h>
#include <vector>

#include <glm/glm.hpp>
#include <SDL2/SDL.h>

class Camera;

namespace sandbox {

enum class Direction{Up, Down, Left, Right};
enum class Selection{Hover, Selected};

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
  void Pan(Direction d, bool on);
  glm::dvec2 GetPanDirection();

  //Geometry control
  const auto& GetPbds() const { return pbds_; }
  pbd::PbdSystem* GetPbd(int idx) const { return pbds_[idx].get(); }
  glm::dvec2 GetPoint(int pdb_idx, int point_idx) const;
  double GetPointRadius() const { return point_radius_; }
  const auto& GetSelections() const { return selections_; }
  void SelectPoint(int pbd_idx, int point_idx, Selection type);
  void SetAttractorPoint(glm::dvec2 p);
  void DeselectAll();

private:
  void HandleFloorCollisions(double dt);
  std::unique_ptr<Camera> camera_;
  std::vector<std::unique_ptr<pbd::PbdSystem>> pbds_;
  std::map<std::pair<int,int>, Selection> selections_;

  glm::dvec2 attractor_point_;
  double floor_level_ = 0.0;
  double time_accumulator_ = 0.0;
  double point_radius_ = 0.01;
  double physics_dt_ = 0.001;
  bool running_ = true;
  pbd::collisions::Collisions collisions_;

  //Camera
  bool pan_left_ = false;
  bool pan_right_ = false;
  bool pan_up_ = false;
  bool pan_down_ = false;
};

}

#endif
