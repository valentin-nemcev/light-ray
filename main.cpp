#include <SDL2/SDL.h>
#include <boost/format.hpp>
#include <iostream>
#include <stdexcept>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

class Display {
    SDL_Window *window = nullptr;
    SDL_Surface *screenSurface = nullptr;

    void sdlError(std::string message) {
        throw std::runtime_error(
            boost::str(boost::format("%s: %s\n") % message % SDL_GetError()));
    }

  public:
    Display() {
        if (SDL_Init(SDL_INIT_VIDEO) > 0) {
            sdlError("Could not initialize SDL2");
        }

        window = SDL_CreateWindow("Light Ray",
                                  SDL_WINDOWPOS_UNDEFINED,
                                  SDL_WINDOWPOS_UNDEFINED,
                                  SCREEN_WIDTH,
                                  SCREEN_HEIGHT,
                                  SDL_WINDOW_SHOWN);
        if (window == nullptr) {
            sdlError("Could not create window");
        }

        screenSurface = SDL_GetWindowSurface(window);
    }

    void setPixel(int x, int y, int value) {
        const SDL_Rect rect = {.x = x, .y = y, .w = 1, .h = 1};
        SDL_FillRect(screenSurface,
                     &rect,
                     SDL_MapRGB(screenSurface->format, value, value, value));
    }

    void update() { SDL_UpdateWindowSurface(window); }

    void waitForQuit() {
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
    }
};

int main(int /*argc*/, char * /*args*/ []) {
    Display display;

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        for (int y = 0; y < SCREEN_HEIGHT; y++) {
            const Uint8 value = ((x / 20) % 2) ^ ((y / 20) % 2);
            display.setPixel(x, y, value * 255);
        }
    }

    display.update();
    display.waitForQuit();

    return 0;
}
