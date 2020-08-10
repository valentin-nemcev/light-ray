#pragma once

#include <boost/format.hpp>
#include <iostream>
#include <span>
#include <string>

#include "SDL.hpp"
#include "renderer.hpp"
#include "stopwatch.hpp"

class Display {
  SDL _sdl;
  SDLWindow _window;
  SDLRenderer _renderer;

  int _screen_pixel_width;
  int _screen_pixel_height;
  SDL_Rect _screen_rect{};
  SDL_Rect _statusbar_rect{};

  SDLTexture _screen_texture;
  SDLTexture _background_texture;

  SDLFont _font;

  bool _is_running = true;

  static constexpr int statusbar_display_height = 20;

  Counter _fps_counter;

public:
  Display(const Display &) = delete;
  Display(Display &&) = delete;
  Display &operator=(const Display &) = delete;
  Display &operator=(Display &&) = delete;

  [[nodiscard]] bool is_running() const { return _is_running; }

  Display(const int window_display_width, const int window_display_height,
          const int display_index)
      : _window(_sdl.create_window(
            window_display_width,
            window_display_height + statusbar_display_height, display_index)),
        _renderer(_window.create_renderer()),

        _screen_pixel_width(window_display_width * _window.display_scale()),
        _screen_pixel_height(window_display_height * _window.display_scale()),

        _screen_rect({.x = 0,
                      .y = 0,
                      .w = _screen_pixel_width,
                      .h = _screen_pixel_height}),
        _statusbar_rect(
            {.x = 0,
             .y = _screen_pixel_height,
             .w = _screen_pixel_width,
             .h = (statusbar_display_height * _window.display_scale())}),
        _screen_texture(_renderer.create_texture(
            _screen_pixel_width, _screen_pixel_height,
            SDL_TEXTUREACCESS_STREAMING, SDL_BLENDMODE_BLEND)),
        _background_texture(_renderer.create_texture(_screen_pixel_width,
                                                     _screen_pixel_height,
                                                     SDL_TEXTUREACCESS_TARGET)),
        _font("../vera_mono.ttf",
              statusbar_display_height * _window.display_scale()) {
    fill_background(_background_texture);
    draw_screen();
    update();

    SDL_EnableScreenSaver(); // It's disabled by default
  }

  ~Display() = default;

  void fill_background(SDLTexture &texture) {
    constexpr int square_size = 16;

    _renderer.to_texture(texture, [](SDLRenderer &renderer) {
      auto [width, height] = renderer.output_size();

      const SDLColor black = {0, 0, 0};
      const SDLColor gray = {32, 32, 32};

      for (int x = 0; x < width / square_size; x++)
        for (int y = 0; y < height / square_size; y++) {
          SDL_Rect rect{.x = x * square_size,
                        .y = y * square_size,
                        .w = square_size,
                        .h = square_size};
          renderer.fill_rect(rect, (x % 2 != y % 2) ? black : gray);
        }
    });
  }

  void draw_screen() {
    _renderer.copy_to(_background_texture, _screen_rect);
    draw_statusbar();
  }

  void draw_pixels(Pixels const &pixels) {
    draw_screen();

    {
      SDLTextureLock texture_lock = _screen_texture.lock();

      for (auto pixel = pixels.begin(); pixel != pixels.end(); ++pixel) {
        if (!is_running())
          break;

        int index = std::distance(pixels.begin(), pixel);

        if (pixel->empty())
          texture_lock.set_i_rgba(index, 0, 0, 0, SDL_ALPHA_TRANSPARENT);
        else
          texture_lock.set_i_rgba(index, pixel->int_r(), pixel->int_g(),
                                  pixel->int_b(), SDL_ALPHA_OPAQUE);
      }
    }
    _renderer.copy_to(_screen_texture, _screen_rect);
  }

  void draw_statusbar() {
    _fps_counter.increment();

    auto fps = _fps_counter.per_second();

    SDLColor bgcolor = {0, 0, 0};
    SDLColor color = {0xff, 0xff, 0xff};

    _renderer.fill_rect(_statusbar_rect, bgcolor);
    _font.text_shaded(_renderer, fps, color, bgcolor, _statusbar_rect.x,
                      _statusbar_rect.y);
  }

  struct ScreenDimensions {
    int width;
    int height;
  };

  ScreenDimensions screen_dimensions() {
    return {.width = _screen_pixel_width, .height = _screen_pixel_height};
  }

  void update() {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      process_event(event);
    }
    _renderer.present();
  }

  void process_event(SDL_Event &event) {
    if (event.type == SDL_QUIT) {
      std::cout << "Exiting..." << std::endl;
      _is_running = false;
    }
  }

  void wait_for_event() {
    SDL_Event event;
    if (SDL_WaitEvent(&event) != 0) {
      process_event(event);
    }
  }
};
