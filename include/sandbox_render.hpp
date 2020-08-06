#include "sandbox.hpp"

namespace sandbox
{

void Render(const Sandbox& s, SDL_Window* window);

glm::ivec2 WorldToPixel(
    const Sandbox& s,
    glm::dvec2 coords,
    SDL_Window* window);

glm::dvec2 PixelToWorld(
    const Sandbox& s,
    SDL_Window* window,
    glm::ivec2 coords);

}
