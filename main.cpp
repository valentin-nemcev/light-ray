#include <iostream>

#include <SDL2/SDL.h>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

void setPixel(SDL_Surface *surface, int x, int y, int value) {
    const SDL_Rect rect = {.x = x, .y = y, .w = 1, .h = 1};
    SDL_FillRect(
        surface, &rect, SDL_MapRGB(surface->format, value, value, value));
}

int main(int /*argc*/, char * /*args*/ []) {
    SDL_Window *window = nullptr;
    SDL_Surface *screenSurface = nullptr;
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
        return 1;
    }
    window = SDL_CreateWindow("Light Ray",
                              SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED,
                              SCREEN_WIDTH,
                              SCREEN_HEIGHT,
                              SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        fprintf(stderr, "could not create window: %s\n", SDL_GetError());
        return 1;
    }
    screenSurface = SDL_GetWindowSurface(window);

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        for (int y = 0; y < SCREEN_HEIGHT; y++) {
            const Uint8 value = ((x / 20) % 2) ^ ((y / 20) % 2);
            setPixel(screenSurface, x, y, value * 255);
        }
    }
    SDL_UpdateWindowSurface(window);

    for (bool running = true; running;) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
