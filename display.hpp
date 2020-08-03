#pragma once

#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <boost/format.hpp>
#include <chrono>
#include <iostream>
#include <string>

#include "renderer.hpp"

class Display {
  SDL_Window *_window = nullptr;
  SDL_Renderer *_renderer = nullptr;

  static void _sdl_error(std::string message) {
    throw std::runtime_error(
        boost::str(boost::format("%s: %s\n") % message % SDL_GetError()));
  }

  std::chrono::time_point<std::chrono::system_clock> _measure_start_time;

  bool _is_running = true;

public:
  Display(const Display &) = delete;
  Display(Display &&) = delete;
  Display &operator=(const Display &) = delete;
  Display &operator=(Display &&) = delete;

  [[nodiscard]] bool is_running() const { return _is_running; }

  Display(const int window_display_width, const int window_display_height,
          const int display_index) {

    if (SDL_Init(SDL_INIT_VIDEO) > 0)
      _sdl_error("Could not initialize SDL2");

    _window = SDL_CreateWindow(
        "Light Ray", SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index),
        SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index), window_display_width,
        window_display_height, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (_window == nullptr)
      _sdl_error("Could not create window");

    _renderer = SDL_CreateRenderer(
        _window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (_renderer == nullptr)
      _sdl_error("Could not create renderer");

    SDL_EnableScreenSaver(); // It's disabled by default
  }

  ~Display() {
    if (_renderer != nullptr)
      SDL_DestroyRenderer(_renderer);
    if (_window != nullptr)
      SDL_DestroyWindow(_window);
    SDL_Quit();
  }

  void draw_background() {
    constexpr int square_size = 16;
    auto [width, height] = screen_dimensions();
    for (int x = 0; x < width / square_size; x++)
      for (int y = 0; y < height / square_size; y++) {
        if (x % 2 != y % 2)
          SDL_SetRenderDrawColor(_renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        else
          SDL_SetRenderDrawColor(_renderer, 32, 32, 32, SDL_ALPHA_OPAQUE);

        SDL_Rect rect{.x = x * square_size,
                      .y = y * square_size,
                      .w = square_size,
                      .h = square_size};
        SDL_RenderFillRect(_renderer, &rect);
      }
  }

  void draw_pixels(Pixels const &pixels) {
    draw_background();

    auto [width, height] = screen_dimensions();

    for (auto pixel = pixels.begin(); pixel != pixels.end(); ++pixel) {
      if (!is_running())
        break;

      if (pixel->empty())
        continue;

      int index = std::distance(pixels.begin(), pixel);

      const Vector2i coord = CameraPixel::index_to_coord(index, width);
      SDL_SetRenderDrawColor(_renderer, pixel->int_r(), pixel->int_g(),
                             pixel->int_b(), SDL_ALPHA_OPAQUE);
      SDL_RenderDrawPoint(_renderer, static_cast<int>(coord.x()),
                          static_cast<int>(coord.y()));
    }
  }

  struct ScreenDimensions {
    int width;
    int height;
  };

  ScreenDimensions screen_dimensions() {
    ScreenDimensions dims{};
    SDL_GetRendererOutputSize(_renderer, &dims.width, &dims.height);
    return dims;
  }

  void update() {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      process_event(event);
    }

    SDL_RenderPresent(_renderer);
  }

  void process_event(SDL_Event &event) {
    if (event.type == SDL_QUIT) {
      _is_running = false;
    }
  }

  void wait_for_event() {
    SDL_Event event;
    if (SDL_WaitEvent(&event) != 0) {
      process_event(event);
    }
  }

  void start_measure() {
    _measure_start_time = std::chrono::system_clock::now();
  }

  void complete_measure(std::string const &message) {

    const auto duration =
        std::chrono::system_clock::now() - _measure_start_time;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std::cout << message << ms << "ms" << std::endl;
  }
};
