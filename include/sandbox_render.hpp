#include "sandbox.hpp"

namespace sandbox
{

void Render(const Sandbox& s, SDL_Window* window);
void RenderAxes(const Sandbox& s, SDL_Window* window);
void Render(
    const Circle* R,
    SDL_Window* window,
    const Camera* camera);

}
