#include "osksdl.hpp"

#include <stdio.h>
#include <SDL2/SDL_image.h>

namespace osksdl
{

double time_;
std::vector<SDL_Window*> windows_;

bool InitSDL()
{
  if(SDL_Init( SDL_INIT_VIDEO ) < 0)
  {
    printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
    return false;
  }

  time_ = 0;
  return true;
}

SDL_Window* CreateWindow(int width, int height, const char* title)
{
  SDL_Window* window =
    SDL_CreateWindow(
      title,
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      width,
      height,
      SDL_WINDOW_SHOWN);

  if(window == nullptr)
  {
    printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
  }

  SDL_Renderer* renderer =
    SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if(renderer == nullptr)
  {
    printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
  }

  if(SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND))
  {
    printf( "Could not set blend mode! SDL Error: %s\n", SDL_GetError() );
  }

  windows_.push_back(window);

  return window;
}

double dt()
{
  const double new_time = 0.001*static_cast<double>(SDL_GetTicks());
  const double dt = new_time-time_;
  time_ = new_time;
  return dt;
}

double GetElapsedTime()
{
  return 0.001*static_cast<double>(SDL_GetTicks());
}

SDL_Texture* LoadTextureFromFile(SDL_Renderer* renderer, std::string file)
{
  SDL_Surface* surf = IMG_Load(file.c_str());
  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_FreeSurface(surf);
  return texture;
}

void QuitSDL()
{
  for (auto& window : windows_)
  {
    SDL_Renderer* renderer = SDL_GetRenderer(window);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
  }
  SDL_Quit();
}



}
