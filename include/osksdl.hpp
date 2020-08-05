#ifndef OSKSDL_H_
#define OSKSDL_H_

#include <vector>
#include <string>
#include <SDL2/SDL.h>
#include <glm/glm.hpp>

class Camera;

namespace osksdl {

bool InitSDL();
void QuitSDL();

SDL_Window* CreateWindow(int width, int height, const char* title);
double dt();
double GetElapsedTime(); //Time in seconds
SDL_Texture* LoadTextureFromFile(SDL_Renderer* renderer, std::string file);


extern double time_;
extern std::vector<SDL_Window*> windows_;

}

#endif
