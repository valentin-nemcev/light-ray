#pragma once

#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <SDL_ttf/SDL_ttf.h>
#include <boost/format.hpp>
#include <chrono>
#include <iostream>
#include <span>
#include <string>

#include "renderer.hpp"

static void sdl_error(std::string message) {
  throw std::runtime_error(
      boost::str(boost::format("%s: %s\n") % message % SDL_GetError()));
}

static void ttf_error(std::string message) {
  throw std::runtime_error(
      boost::str(boost::format("%s: %s\n") % message % TTF_GetError()));
}

class TextureLock {
  SDL_Texture &_texture;
  SDL_PixelFormat *_pixel_format = nullptr;
  std::span<Uint32> _texture_span;
  int _width = 0;
  int _height = 0;

public:
  TextureLock(const TextureLock &) = delete;
  TextureLock(TextureLock &&) = delete;
  TextureLock &operator=(const TextureLock &) = delete;
  TextureLock &operator=(TextureLock &&) = delete;

  TextureLock(SDL_Texture &texture) : _texture(texture) {

    Uint32 format_tag = 0;

    int access = 0;
    SDL_QueryTexture(&_texture, &format_tag, &access, &_width, &_height);

    _pixel_format = SDL_AllocFormat(format_tag);
    if (_pixel_format == nullptr)
      sdl_error("Could not allocate pixel format");

    void *texture_pixels = nullptr;
    int byte_pitch = 0;
    if (SDL_LockTexture(&_texture, nullptr, &texture_pixels, &byte_pitch) != 0)
      sdl_error("Could not lock screen texture for updating");

    _texture_span =
        std::span(static_cast<Uint32 *>(texture_pixels), _width * _height);
  }

  ~TextureLock() {
    SDL_UnlockTexture(&_texture);
    if (_pixel_format != nullptr) {
      SDL_FreeFormat(_pixel_format);
    }
  }

  void set_xy_rgba(int x, int y, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    _texture_span[x + (y * _width)] = SDL_MapRGBA(_pixel_format, r, g, b, a);
  }

  void set_i_rgba(int i, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    _texture_span[i] = SDL_MapRGBA(_pixel_format, r, g, b, a);
  }
};

class Display {
  SDL_Window *_window = nullptr;
  SDL_Renderer *_renderer = nullptr;
  SDL_Texture *_screen_texture = nullptr;
  SDL_Texture *_background_texture = nullptr;
  SDL_PixelFormat *_pixel_format = nullptr;

  TTF_Font *_font;

  std::chrono::time_point<std::chrono::system_clock> _measure_start_time;

  bool _is_running = true;

  int _screen_pixel_width;
  int _screen_pixel_height;
  SDL_Rect _screen_rect{};
  SDL_Rect _statusbar_rect{};

  static constexpr int statusbar_display_height = 20;

public:
  Display(const Display &) = delete;
  Display(Display &&) = delete;
  Display &operator=(const Display &) = delete;
  Display &operator=(Display &&) = delete;

  [[nodiscard]] bool is_running() const { return _is_running; }

  Display(const int window_display_width, const int window_display_height,
          const int display_index) {

    if (SDL_Init(SDL_INIT_VIDEO) > 0)
      sdl_error("Could not initialize SDL2");

    _window = SDL_CreateWindow(
        "Light Ray", SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index),
        SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index), window_display_width,
        window_display_height + statusbar_display_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (_window == nullptr)
      sdl_error("Could not create window");

    _renderer = SDL_CreateRenderer(
        _window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (_renderer == nullptr)
      sdl_error("Could not create renderer");

    _pixel_format = SDL_AllocFormat(SDL_GetWindowPixelFormat(_window));
    if (_pixel_format == nullptr)
      sdl_error("Could not allocate pixel format");

    int window_pixel_width = 0;
    int window_pixel_height = 0;
    SDL_GetRendererOutputSize(_renderer, &window_pixel_width,
                              &window_pixel_height);
    int display_scale = window_pixel_width / window_display_width;

    _screen_pixel_width = window_display_width * display_scale;
    _screen_pixel_height = window_display_height * display_scale;

    _screen_rect = {
        .x = 0, .y = 0, .w = _screen_pixel_width, .h = _screen_pixel_height};
    _statusbar_rect = {.x = 0,
                       .y = _screen_pixel_height,
                       .w = _screen_pixel_width,
                       .h = (statusbar_display_height * display_scale)};

    _screen_texture = SDL_CreateTexture(
        _renderer, SDL_GetWindowPixelFormat(_window),
        SDL_TEXTUREACCESS_STREAMING, _screen_pixel_width, _screen_pixel_height);
    if (_screen_texture == nullptr)
      sdl_error("Could not create screen texture");
    if (SDL_SetTextureBlendMode(_screen_texture, SDL_BLENDMODE_BLEND) != 0)
      sdl_error("Could not set screen texture blend mode");

    _background_texture = SDL_CreateTexture(
        _renderer, SDL_GetWindowPixelFormat(_window), SDL_TEXTUREACCESS_TARGET,
        _screen_pixel_width, _screen_pixel_height);
    if (_background_texture == nullptr)
      sdl_error("Could not create background texture");
    fill_background(*_background_texture);

    SDL_EnableScreenSaver(); // It's disabled by default

    if (TTF_Init() != 0)
      ttf_error("Could not initialize SDL_ttf");

    // load font.ttf at size 16 into font
    _font = TTF_OpenFont("../vera_mono.ttf",
                         statusbar_display_height * display_scale);
    if (_font == nullptr)
      ttf_error("Could not load font");
  }

  ~Display() {

    if (_font != nullptr)
      TTF_CloseFont(_font);

    TTF_Quit();

    if (_background_texture != nullptr)
      SDL_DestroyTexture(_background_texture);
    if (_screen_texture != nullptr)
      SDL_DestroyTexture(_screen_texture);
    if (_pixel_format != nullptr)
      SDL_FreeFormat(_pixel_format);
    if (_renderer != nullptr)
      SDL_DestroyRenderer(_renderer);
    if (_window != nullptr)
      SDL_DestroyWindow(_window);
    SDL_Quit();
  }

  void fill_background(SDL_Texture &texture) {
    constexpr int square_size = 16;
    if (SDL_SetRenderTarget(_renderer, &texture) != 0)
      sdl_error("Could not set render target to texture");

    int width = 0;
    int height = 0;
    SDL_GetRendererOutputSize(_renderer, &width, &height);

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
    SDL_SetRenderTarget(_renderer, nullptr);
  }

  void draw_pixels(Pixels const &pixels) {
    if (SDL_RenderCopy(_renderer, _background_texture, nullptr,
                       &_screen_rect) != 0)
      sdl_error("Could not copy background texture");

    {
      TextureLock texture_lock(*_screen_texture);

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
    if (SDL_RenderCopy(_renderer, _screen_texture, nullptr, &_screen_rect) != 0)
      sdl_error("Could not copy screen texture");

    draw_statusbar();
  }

  void draw_statusbar() {
    SDL_Color bgcolor = {0, 0, 0, SDL_ALPHA_OPAQUE};
    SDL_Color color = {0xff, 0xff, 0xff, SDL_ALPHA_OPAQUE};

    if (SDL_SetRenderDrawColor(_renderer, bgcolor.r, bgcolor.g, bgcolor.b,
                               bgcolor.a) != 0)
      sdl_error("Could not set renderer color");
    if (SDL_RenderFillRect(_renderer, &_statusbar_rect) != 0)
      sdl_error("Could not fill statusbar rect");

    SDL_Surface *text_surface =
        TTF_RenderText_Shaded(_font, "Hello World!", color, bgcolor);
    if (text_surface == nullptr)
      ttf_error("Could not render text");

    SDL_Texture *text_texture =
        SDL_CreateTextureFromSurface(_renderer, text_surface);
    if (text_texture == nullptr)
      sdl_error("Could not create text texture");

    SDL_Rect target_rect{.x = _statusbar_rect.x,
                         .y = _statusbar_rect.y,
                         .w = text_surface->w,
                         .h = text_surface->h};

    if (SDL_RenderCopy(_renderer, text_texture, nullptr, &target_rect) != 0)
      sdl_error("Could not copy text texture");

    SDL_DestroyTexture(text_texture);
    SDL_FreeSurface(text_surface);
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
