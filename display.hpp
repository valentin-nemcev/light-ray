#pragma once

#include <boost/format.hpp>
#include <iostream>
#include <string>

#include "SDL.hpp"
#include "renderer.hpp"
#include "statistics.hpp"
#include "stopwatch.hpp"

class Statusbar {
  SDLRenderer &_renderer;
  int _font_size;
  SDLFont _font;

  int _padding;
  SDL_Rect _padded_rect{};
  SDL_Rect _rect{};
  int _baseline;

  Counter _fps_counter;

  static constexpr int display_padding = 4;

  SDLColor _bgcolor = {0, 0, 0};
  SDLColor _color = {0xff, 0xff, 0xff};

public:
  Statusbar(const Statusbar &) = delete;
  Statusbar(Statusbar &&) = delete;
  Statusbar &operator=(const Statusbar &) = delete;
  Statusbar &operator=(Statusbar &&) = delete;
  ~Statusbar() = default;

  Statusbar(int font_display_size, SDLWindow &window, SDLRenderer &renderer)
      : _renderer(renderer),
        _font_size(font_display_size * window.display_scale()),
        _font("../liberation_mono.ttf", _font_size),
        _padding(display_padding * window.display_scale()),
        _padded_rect({.x = 0,
                      .y = window.pixel_size().height,
                      .w = window.pixel_size().width,
                      .h = _font.height() + _padding * 2}),
        _rect({.x = _padded_rect.x + _padding,
               .y = _padded_rect.y + _padding,
               .w = _padded_rect.w - _padding,
               .h = _padded_rect.h - _padding}),
        _baseline(_rect.y + _font.ascent()) {
    window.pixel_resize(
        {.width = window.pixel_size().width,
         .height = window.pixel_size().height + _padded_rect.h});
  }

  int draw_histogram(int x, Histogram &histogram) {
    auto buckets = histogram.normalized_buckets();
    int size = static_cast<int>(buckets.size());
    if (size == 0)
      return x;
    int width = _font.width() * 4 / size;

    _renderer.fill_rect(
        {.x = _rect.x + x, .y = _baseline, .w = width * size, .h = 1}, _color);
    for (auto val : buckets) {
      int height =
          static_cast<int>(std::round(static_cast<float>(_font_size) * val));
      _renderer.fill_rect(
          {.x = _rect.x + x, .y = _baseline - height, .w = width, .h = height},
          _color);
      x += width;
    }
    return x;
  }

  int draw_text(int x, const std::string &text) {
    return x + _font.text_shaded(_renderer, text, _color, _bgcolor,
                                 {.x = _rect.x + x, .y = _rect.y});
  }

  void clear() { _renderer.fill_rect(_padded_rect, _bgcolor); }

  int draw(Statistics &pixel_stats,
           std::optional<PixelDisplayValue> current_pixel,
           long cpu_seconds_spent) {
    _fps_counter.increment();

    auto fps = _fps_counter.per_second();

    clear();

    int x = 0;
    x = draw_histogram(x, pixel_stats.value_histogram());
    x += _font.width() * 2;
    x = draw_text(x, fps);

    x += _font.width() * 2;
    x = draw_text(x, boost::str(boost::format("%ds") % cpu_seconds_spent));
    x += _font.width() * 2;
    x = draw_text(
        x, boost::str(boost::format("%d") %
                      (current_pixel ? current_pixel->iterations
                                     : pixel_stats.iterations_per_pixel())));

    if (current_pixel) {
      x += _font.width();
      x = draw_text(x, boost::str(boost::format("%3d") %
                                  static_cast<int>(current_pixel->red)));
    }
    x += _font.width();
    x = draw_text(x, boost::str(boost::format("%5.3f") %
                                (current_pixel ? current_pixel->value
                                               : pixel_stats.avg_value())));
    x += _font.width();
    x = draw_text(x, boost::str(boost::format("%5.3f") %
                                (current_pixel ? current_pixel->std_dev
                                               : pixel_stats.std_dev())));
    x += _font.width();
    x = draw_text(x, boost::str(boost::format("%7.5f") %
                                (current_pixel ? current_pixel->std_error
                                               : pixel_stats.avg_std_error())));
    return x;
  }
};

class Display {
  SDL _sdl;

  SDLWindow _window;
  SDLRenderer _renderer;

  SDLRect _screen_rect;

  Statusbar _statusbar;

  SDLTexture _screen_texture;
  SDLTexture _background_texture;

  bool _is_running = true;

  Statistics _pixel_stats;

  long _cpu_seconds_spent = 0;

  Pixels *_pixels_ptr = nullptr;

public:
  Display(const Display &) = delete;
  Display(Display &&) = delete;
  Display &operator=(const Display &) = delete;
  Display &operator=(Display &&) = delete;

  [[nodiscard]] bool is_running() const { return _is_running; }

  Display(const int window_display_width, const int window_display_height,
          const int display_index)
      : _window(_sdl.create_window(window_display_width, window_display_height,
                                   display_index)),
        _renderer(_window.create_renderer()),

        _screen_rect(
            {.x = 0, .y = 0},
            {.width = window_display_width * _window.display_scale(),
             .height = window_display_height * _window.display_scale()}),
        _statusbar(16, _window, _renderer),
        _screen_texture(_renderer.create_texture(_screen_rect.size(),
                                                 SDL_TEXTUREACCESS_STREAMING,
                                                 SDL_BLENDMODE_BLEND)),
        _background_texture(_renderer.create_texture(
            _screen_rect.size(), SDL_TEXTUREACCESS_TARGET)) {
    fill_background(_background_texture);
    draw_screen();
    update();
  }

  ~Display() = default;

  void set_pixels(Pixels *pixels_ptr) { _pixels_ptr = pixels_ptr; }

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

  void set_stats(long cpu_seconds_spent) {
    _cpu_seconds_spent = cpu_seconds_spent;
  }

  void draw_screen() {
    _renderer.copy_to(_background_texture, _screen_rect);
    _statusbar.clear();
  }

  void draw_pixels() {
    _renderer.copy_to(_background_texture, _screen_rect);

    _pixel_stats = Statistics();
    {
      SDLTextureLock texture_lock = _screen_texture.lock();

      for (size_t index = 0; index < _pixels_ptr->size(); index++) {
        if (!is_running())
          break;

        CameraPixel &pixel = (*_pixels_ptr)[index];

        if (pixel.empty())
          texture_lock.set_i_rgba(index, SDLColor::transparent);
        else {
          auto display_value = pixel.display_value();
          texture_lock.set_i_rgba(index, {.red = display_value.red,
                                          .green = display_value.green,
                                          .blue = display_value.blue});
          _pixel_stats.count_pixel(display_value);
        }
      }
    }
    _renderer.copy_to(_screen_texture, _screen_rect);
    auto mouse_pos = _window.pixel_mouse_pos();
    auto display_value =
        mouse_pos && _screen_rect.contains(*mouse_pos)
            ? std::optional<PixelDisplayValue>(
                  _pixels_ptr->at(mouse_pos->index(_screen_rect.size()))
                      .display_value())
            : std::nullopt;
    _statusbar.draw(_pixel_stats, display_value, _cpu_seconds_spent);
  }

  SDLSize screen_dimensions() { return _screen_rect.size(); }

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
};
