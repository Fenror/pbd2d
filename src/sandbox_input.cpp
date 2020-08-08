#include "sandbox_input.hpp"

#include "camera.hpp"
#include "sandbox.hpp"
#include "sandbox_render.hpp"

namespace sandbox
{

void KeyDown(Sandbox* s, SDL_Keycode sym)
{
  switch (sym)
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

void KeyUp(Sandbox* s, SDL_Keycode sym)
{
  switch (sym)
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

void PixelDistanceToWorldCoordinate(
    const Sandbox& s, SDL_Window* w,
    glm::ivec2 pixel_coords,
    glm::dvec2 world_coords,
    int* dx, int* dy)
{
  const auto target_pixel = WorldToPixel(s, world_coords, w);
  *dx = glm::abs(target_pixel.x - pixel_coords.x);
  *dy = glm::abs(target_pixel.y - pixel_coords.y);
}

void SelectNearestPoint(
    Sandbox* s,
    SDL_Window* w,
    glm::ivec2 pixel,
    int tol)
{
  const auto& pbds = s->GetPbds();
  const auto& selections = s->GetSelections();

  for (int pbd_idx = 0; pbd_idx < pbds.size(); ++pbd_idx)
  {
    const auto& pbd = pbds[pbd_idx];
    const int num_pts = pbd->GetNumPoints();
    for (int point_idx = 0; point_idx < num_pts; ++point_idx)
    {
      const auto p = pbd->GetPoint(point_idx);
      const auto point_pixel = WorldToPixel(*s, p, w);
      if (glm::abs(point_pixel.x-pixel.x) < tol &&
          glm::abs(point_pixel.y-pixel.y) < tol)
      {
        s->SelectPoint(pbd_idx, point_idx, Selection::Selected);
        return;
      }
    }
  }
}

void MouseDown(Sandbox* s, SDL_Window* w, int x, int y)
{
  int tol = 20;
  s->SetAttractorPoint(PixelToWorld(*s, w, {x,y}));
  SelectNearestPoint(s, w, {x,y}, tol);
}

void MouseUp(Sandbox* s)
{
  s->DeselectAll();
}

void MouseMove(Sandbox* s, SDL_Window* w, int x, int y)
{
  s->SetAttractorPoint(PixelToWorld(*s, w, {x,y}));
}

void Scroll(Sandbox* s, int y)
{
  const auto z = glm::exp(0.1*static_cast<double>(y));
  s->GetCamera()->Zoom(z);
}

void EventHandler(Sandbox* s, SDL_Event e)
{
  switch(e.key.type)
  {
    case SDL_KEYDOWN:
      KeyDown(s, e.key.keysym.sym);
      break;
    case SDL_KEYUP:
      KeyUp(s, e.key.keysym.sym);
      break;
  }

  const auto w = SDL_GetWindowFromID(e.button.windowID);
  switch(e.button.type)
  {
    case SDL_MOUSEBUTTONDOWN:
      MouseDown(s, w, e.button.x, e.button.y);
      break;
    case SDL_MOUSEBUTTONUP:
      s->DeselectAll();
      break;
  }

  MouseMove(s, w, e.motion.x, e.motion.y);

  if (e.type == SDL_MOUSEWHEEL)
  {
    Scroll(s, e.wheel.y);
  }
}

}
