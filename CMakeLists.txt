cmake_minimum_required(VERSION 3.8)
project(pbd2d LANGUAGES CXX C)

add_library(fontcache STATIC src/SDL_FontCache.c)
target_include_directories(fontcache PUBLIC include)

add_library(pbd2d STATIC src/point_cloud.cpp
                         src/constraints.cpp
                         src/pbd_system.cpp
                         src/pbd_factory.cpp)
target_include_directories(pbd2d PUBLIC include)

add_executable(main src/camera.cpp
                    src/circle.cpp
                    src/main.cpp
                    src/osksdl.cpp
                    src/sandbox.cpp
                    src/sandbox_render.cpp)
target_include_directories(main PUBLIC include)
target_link_libraries(main PUBLIC -lSDL2 -lSDL2_image -lSDL2_gfx -lSDL2_ttf fontcache pbd2d)
target_compile_features(main PUBLIC cxx_std_17)
