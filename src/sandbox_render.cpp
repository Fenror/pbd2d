#include "sandbox_render.hpp"

#include <SDL2/SDL2_gfxPrimitives.h>

#include "sandbox.hpp"
#include "camera.hpp"
#include "dynamical_system.hpp"
#include "rod.hpp"

namespace sandbox
{

namespace {
  glm::dvec2 WorldToNDC(
      glm::dvec2 coords,
      const Camera* camera)
  {
    double left, right, top, bottom;
    camera->GetLimits(&left, &right, &top, &bottom);
    glm::dvec2 local = coords - camera->GetCenter();
    glm::dvec2 ndc{local.x/(right-left) + 0.5,
                   local.y/(top-bottom) + 0.5};

    return ndc;
  }

  glm::ivec2 NDCToPixel(glm::dvec2 coords, SDL_Window* window)
  {
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    int x = static_cast<int>(coords.x*width);
    int y = static_cast<int>((1.0-coords.y)*height);

    return glm::ivec2{x,y};
  }

  glm::ivec2 WorldToPixel(
      const Sandbox& s,
      glm::dvec2 coords,
      SDL_Window* window)
  {
    auto camera = s.GetCamera();
    glm::dvec2 ndc = WorldToNDC(coords, camera);
    glm::ivec2 pixel = NDCToPixel(ndc, window);
    return pixel;
  }

  glm::dvec2 PixelToNDC(glm::ivec2 coords, SDL_Window* window)
  {
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    glm::dvec2 ndc;
    ndc.x = static_cast<double>(coords.x)/width;
    ndc.y = 1.0 - static_cast<double>(coords.y)/height;

    return ndc;
  }

  glm::dvec2 NDCToWorld(glm::dvec2 coords, const Camera* camera)
  {
    double left, right, top, bottom;
    camera->GetLimits(&left, &right, &top, &bottom);
    glm::dvec2 world;
    world.x = left + coords.x*(right-left);
    world.y = bottom + coords.y*(top-bottom);

    return world;
  }

  glm::dvec2 PixelToWorld(
      glm::ivec2 coords,
      SDL_Window* window,
      const Camera* camera)
  {
    glm::dvec2 ndc = PixelToNDC(coords, window);
    glm::dvec2 world = NDCToWorld(ndc, camera);

    return world;
  }

  short WorldLengthToPixelLength(
      const Sandbox& s,
      double world_length,
      SDL_Window* window)
  {
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    auto camera = s.GetCamera();
    const short pixel_length =
        static_cast<short>(world_length / camera->GetWidth() * width);
    return pixel_length;
  }
}


void RenderAxes(const Sandbox& s, SDL_Window* window)
{
  auto origin = WorldToPixel(s, {0.0,0.0}, window);
  auto renderer = SDL_GetRenderer(window);
  int w,h;
  SDL_GetWindowSize(window, &w, &h);
  Uint8 r,g,b,a;
  SDL_GetRenderDrawColor(renderer,&r,&g,&b,&a);
  hlineRGBA(renderer, 0, w, origin.y, 0, 0, 0, 255);
  vlineRGBA(renderer, origin.x, 0, h, 0, 0, 0, 255);
  SDL_SetRenderDrawColor(renderer,r,g,b,a);
}

void RenderDynamicalSystem(
    const Sandbox& s,
    const DynamicalSystem* d,
    SDL_Window* window)
{
  auto points = d->GetPoints();

  auto renderer = SDL_GetRenderer(window);
  Uint8 r,g,b,a;
  SDL_GetRenderDrawColor(renderer,&r,&g,&b,&a);
  for (const auto& p : points)
  {
    auto center = WorldToPixel(s, p, window);
    auto radius = WorldLengthToPixelLength(s, s.GetPointRadius(), window);
    filledCircleRGBA(renderer, center.x, center.y, radius, 255, 0, 0, 255);
  }
  SDL_SetRenderDrawColor(renderer,r,g,b,a);
}

void Render(const Sandbox& s, SDL_Window* window)
{
  auto renderer = SDL_GetRenderer(window);
  SDL_RenderClear(renderer);
  SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
  RenderAxes(s, window);
  //RenderDynamicalSystem(s, s.GetDynamicalSystem(), window);
  RenderDynamicalSystem(s, s.GetRod()->GetDynamicalSystem(), window);
  SDL_RenderPresent(renderer);
}

}
