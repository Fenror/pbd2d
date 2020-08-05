#include "sandbox.hpp"

namespace sandbox
{

void Render(const Sandbox& s, SDL_Window* window);
void Render(
    const Circle* R,
    SDL_Window* window,
    const Camera* camera);
glm::dvec2 WorldToNDC(glm::dvec2 coords);
glm::ivec2 NDCToPixel(glm::dvec2 coords);
glm::ivec2 WorldToPixel(
    glm::dvec2 coords,
    SDL_Window* window,
    const Camera* camera);
glm::dvec2 PixelToNDC(glm::ivec2 coords);
glm::dvec2 NDCToWorld(glm::dvec2 coords);
glm::dvec2 PixelToWorld(
    glm::ivec2 coords,
    SDL_Window* window,
    const Camera* camera);
short WorldLengthToPixelLength(
    double l,
    SDL_Window* window,
    const Camera* camera);

}
