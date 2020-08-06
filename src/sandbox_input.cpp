#include "sandbox_input.hpp"

#include "sandbox.hpp"

namespace sandbox
{

namespace
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
}

}
