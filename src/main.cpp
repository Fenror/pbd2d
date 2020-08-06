#include <iostream>
#include "sandbox.hpp"
#include "sandbox_render.hpp"
#include "sandbox_input.hpp"

#include "osksdl.hpp"

int main(int argc, char *argv[])
{
    osksdl::InitSDL();
    SDL_Rect gScreenRect = { 0, 0, 320, 240 };
    SDL_DisplayMode displayMode;
    if( SDL_GetCurrentDisplayMode( 0, &displayMode ) == 0 )
    {
        gScreenRect.w = displayMode.w;
        gScreenRect.h = displayMode.h;
    }
    SDL_Window* window = osksdl::CreateWindow(gScreenRect.w, gScreenRect.h, "test");

    sandbox::Sandbox game;

    bool quit = false;
    SDL_Event e;

    while(game.IsRunning())
    {
        game.UpdateDynamics(osksdl::dt());
        sandbox::Render(game, window);

        while( SDL_PollEvent( &e ) != 0 )
        {
            sandbox::EventHandler(&game, e);
        }
    }

    osksdl::QuitSDL();

    return 0;
}
