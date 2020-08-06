#ifndef SANDBOX_INPUT_H_
#define SANDBOX_INPUT_H_

#include <SDL2/SDL.h>

namespace sandbox
{

class Sandbox;

void EventHandler(Sandbox* s, SDL_Event e);

}

#endif
