#pragma once

#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <SDL_ttf/SDL_ttf.h>
#include <boost/format.hpp>
#include <iostream>
#include <span>

static void sdl_error(std::string message) {
  throw std::runtime_error(
      boost::str(boost::format("%s: %s\n") % message % SDL_GetError()));
}

static void ttf_error(std::string message) {
  throw std::runtime_error(
      boost::str(boost::format("%s: %s\n") % message % TTF_GetError()));
}

struct SDLSize {
  int width = 0;
  int height = 0;
};

struct SDLColor {
  Uint8 r = 0;
  Uint8 g = 0;
  Uint8 b = 0;
  Uint8 a = SDL_ALPHA_OPAQUE;
};

class SDLWindow;

class SDL {

public:
  SDL(const SDL &) = delete;
  SDL(SDL &&) = delete;
  SDL &operator=(const SDL &) = delete;
  SDL &operator=(SDL &&) = delete;

  SDL() {
    if (SDL_Init(SDL_INIT_VIDEO) > 0)
      sdl_error("Could not initialize SDL2");
    if (TTF_Init() != 0)
      ttf_error("Could not initialize SDL_ttf");
  }

  ~SDL() {
    TTF_Quit();
    SDL_Quit();
  }

  SDLWindow create_window(int window_display_width, int window_display_height,
                          int display_index);
};

class SDLRenderer;
class SDLTextureRenderer;
class SDLTextureLock;

using SDLPixelFormat = Uint32;

class SDLTexture {
  SDL_Texture *_texture;

public:
  SDLTexture(const SDLTexture &) = delete;
  SDLTexture(SDLTexture &&) = delete;
  SDLTexture &operator=(const SDLTexture &) = delete;
  SDLTexture &operator=(SDLTexture &&) = delete;

  SDLTexture(SDL_Renderer *renderer_sdl_ptr, SDLPixelFormat pixel_format,
             SDLSize size, int access, SDL_BlendMode blend_mode);
  SDLTexture(SDL_Renderer *renderer_sdl_ptr, SDL_Surface *surface_sdl_ptr,
             SDL_BlendMode blend_mode);

  ~SDLTexture();
  SDL_Texture *sdl_ptr() { return _texture; }

  SDLTextureLock lock();
};

class SDLWindow {
  SDL_Window *_window_sdl_ptr;
  SDLSize _pixel_size;
  int _display_scale = 0;

public:
  [[nodiscard]] int display_scale() const { return _display_scale; }
  [[nodiscard]] SDLSize pixel_size() const { return _pixel_size; }

  SDLWindow(const SDLWindow &) = delete;
  SDLWindow(SDLWindow &&) = delete;
  SDLWindow &operator=(const SDLWindow &) = delete;
  SDLWindow &operator=(SDLWindow &&) = delete;

  SDLWindow(const int display_width, const int display_height,
            const int display_index) {
    _window_sdl_ptr = SDL_CreateWindow(
        "Light Ray", SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index),
        SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index), display_width,
        display_height, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (_window_sdl_ptr == nullptr)
      sdl_error("Could not create window");

    SDL_GL_GetDrawableSize(_window_sdl_ptr, &_pixel_size.width,
                           &_pixel_size.height);
    _display_scale = _pixel_size.width / display_width;
  }

  ~SDLWindow() {
    if (_window_sdl_ptr != nullptr)
      SDL_DestroyWindow(_window_sdl_ptr);
  }

  void pixel_resize(const SDLSize &size) {
    SDL_SetWindowSize(_window_sdl_ptr, size.width / display_scale(),
                      size.height / display_scale());
  }

  SDLRenderer create_renderer();
  SDLTextureRenderer create_texture_renderer(SDLTexture &texture);
};

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
SDLWindow SDL::create_window(const int window_display_width,
                             const int window_display_height,
                             const int display_index) {
  return SDLWindow(window_display_width, window_display_height, display_index);
}

class SDLRenderer {
  SDL_Renderer *_renderer_sdl_ptr;
  SDLPixelFormat _pixel_format;

public:
  SDLRenderer(const SDLRenderer &) = delete;
  SDLRenderer(SDLRenderer &&) = delete;
  SDLRenderer &operator=(const SDLRenderer &) = delete;
  SDLRenderer &operator=(SDLRenderer &&) = delete;

  SDLRenderer(SDL_Window *window_ptr) {
    _renderer_sdl_ptr = SDL_CreateRenderer(
        window_ptr, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (_renderer_sdl_ptr == nullptr)
      sdl_error("Could not create renderer");

    _pixel_format = SDL_GetWindowPixelFormat(window_ptr);
  }

  ~SDLRenderer() {
    if (_renderer_sdl_ptr != nullptr)
      SDL_DestroyRenderer(_renderer_sdl_ptr);
  }

  [[nodiscard]] SDLPixelFormat pixel_format() const { return _pixel_format; }

  [[nodiscard]] SDLSize output_size() const {
    SDLSize size;
    SDL_GetRendererOutputSize(_renderer_sdl_ptr, &size.width, &size.height);
    return size;
  }

  void to_viewport(SDL_Rect &viewport,
                   const std::function<void(SDLRenderer &)> &callback) {

    if (SDL_RenderSetViewport(_renderer_sdl_ptr, &viewport) != 0)
      sdl_error("Could not set render viewport");
    callback(*this);
    if (SDL_RenderSetViewport(_renderer_sdl_ptr, nullptr) != 0)
      sdl_error("Could not reset render viewport");
  }

  void to_texture(SDLTexture &texture,
                  const std::function<void(SDLRenderer &)> &callback) {

    if (SDL_SetRenderTarget(_renderer_sdl_ptr, texture.sdl_ptr()) != 0)
      sdl_error("Could not set render target to texture");
    callback(*this);
    if (SDL_SetRenderTarget(_renderer_sdl_ptr, nullptr) != 0)
      sdl_error("Could not set render target to window");
  }

  SDLTexture create_texture(SDLSize size, SDL_TextureAccess access,
                            SDL_BlendMode blend_mode = SDL_BLENDMODE_NONE);
  SDLTexture create_texture(SDL_Surface *surface_ptr,
                            SDL_BlendMode blend_mode = SDL_BLENDMODE_NONE);

  void fill_rect(const SDL_Rect &rect, SDLColor color) {
    SDL_SetRenderDrawColor(_renderer_sdl_ptr, color.r, color.g, color.b,
                           color.a);
    SDL_RenderFillRect(_renderer_sdl_ptr, &rect);
  }

  void copy_to(SDLTexture &texture, SDL_Rect target_rect) {
    if (SDL_RenderCopy(_renderer_sdl_ptr, texture.sdl_ptr(), nullptr,
                       &target_rect) != 0)
      sdl_error("Could not copy background texture");
  }

  void present() { SDL_RenderPresent(_renderer_sdl_ptr); }
};

SDLRenderer SDLWindow::create_renderer() {
  return SDLRenderer(_window_sdl_ptr);
}

SDLTexture::SDLTexture(SDL_Renderer *renderer_sdl_ptr,
                       SDLPixelFormat pixel_format, SDLSize size, int access,
                       SDL_BlendMode blend_mode) {
  _texture = SDL_CreateTexture(renderer_sdl_ptr, pixel_format, access,
                               size.width, size.height);
  if (_texture == nullptr)
    sdl_error("Could not create texture");

  if (SDL_SetTextureBlendMode(_texture, blend_mode) != 0)
    sdl_error("Could not set screen texture blend mode");
}

SDLTexture::SDLTexture(SDL_Renderer *renderer_sdl_ptr,
                       SDL_Surface *surface_sdl_ptr, SDL_BlendMode blend_mode) {

  _texture = SDL_CreateTextureFromSurface(renderer_sdl_ptr, surface_sdl_ptr);
  if (_texture == nullptr)
    sdl_error("Could not create texture");

  if (SDL_SetTextureBlendMode(_texture, blend_mode) != 0)
    sdl_error("Could not set screen texture blend mode");
}

SDLTexture::~SDLTexture() {
  if (_texture != nullptr)
    SDL_DestroyTexture(_texture);
}
SDLTexture SDLRenderer::create_texture(SDLSize size, SDL_TextureAccess access,
                                       SDL_BlendMode blend_mode) {
  return SDLTexture(_renderer_sdl_ptr, pixel_format(), size, access,
                    blend_mode);
}
SDLTexture SDLRenderer::create_texture(SDL_Surface *surface_ptr,
                                       SDL_BlendMode blend_mode) {
  return SDLTexture(_renderer_sdl_ptr, surface_ptr, blend_mode);
}

class SDLTextureLock {
  SDL_Texture *_texture_sdl_ptr;
  SDL_PixelFormat *_pixel_format_sdl_ptr = nullptr;
  std::span<Uint32> _texture_span;
  int _width = 0;
  int _height = 0;

public:
  SDLTextureLock(const SDLTextureLock &) = delete;
  SDLTextureLock(SDLTextureLock &&) = delete;
  SDLTextureLock &operator=(const SDLTextureLock &) = delete;
  SDLTextureLock &operator=(SDLTextureLock &&) = delete;

  SDLTextureLock(SDLTexture &texture) : _texture_sdl_ptr(texture.sdl_ptr()) {

    Uint32 format_tag = 0;

    int access = 0;
    SDL_QueryTexture(_texture_sdl_ptr, &format_tag, &access, &_width, &_height);

    _pixel_format_sdl_ptr = SDL_AllocFormat(format_tag);
    if (_pixel_format_sdl_ptr == nullptr)
      sdl_error("Could not allocate pixel format");

    void *texture_pixels_ptr = nullptr;
    int byte_pitch = 0;
    if (SDL_LockTexture(_texture_sdl_ptr, nullptr, &texture_pixels_ptr,
                        &byte_pitch) != 0)
      sdl_error("Could not lock screen texture for updating");

    _texture_span =
        std::span(static_cast<Uint32 *>(texture_pixels_ptr), _width * _height);
  }

  ~SDLTextureLock() {
    SDL_UnlockTexture(_texture_sdl_ptr);
    if (_pixel_format_sdl_ptr != nullptr) {
      SDL_FreeFormat(_pixel_format_sdl_ptr);
    }
  }

  void set_xy_rgba(int x, int y, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    _texture_span[x + (y * _width)] =
        SDL_MapRGBA(_pixel_format_sdl_ptr, r, g, b, a);
  }

  void set_i_rgba(int i, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    _texture_span[i] = SDL_MapRGBA(_pixel_format_sdl_ptr, r, g, b, a);
  }
};

SDLTextureLock SDLTexture::lock() { return SDLTextureLock(*this); };

class SDLFont {
  TTF_Font *_font_sdl_ptr = nullptr;

  [[nodiscard]] static SDL_Color _sdl_color(const SDLColor color) {
    return {.r = color.r, .g = color.g, .b = color.b, .a = color.a};
  }

  int _height;
  int _ascent;
  int _width;

public:
  [[nodiscard]] int height() const { return _height; }
  [[nodiscard]] int ascent() const { return _ascent; }
  [[nodiscard]] int width() const { return _width; }

  SDLFont(const SDLFont &) = delete;
  SDLFont(SDLFont &&) = delete;
  SDLFont &operator=(const SDLFont &) = delete;
  SDLFont &operator=(SDLFont &&) = delete;

  SDLFont(const std::string &font_file, int size) {
    _font_sdl_ptr = TTF_OpenFont(font_file.c_str(), size);
    if (_font_sdl_ptr == nullptr)
      ttf_error("Could not load font");

    TTF_SetFontHinting(_font_sdl_ptr, TTF_HINTING_LIGHT);

    _height = TTF_FontHeight(_font_sdl_ptr);
    _ascent = TTF_FontAscent(_font_sdl_ptr);
    _width = text_width(" ");
  };

  ~SDLFont() {

    if (_font_sdl_ptr != nullptr)
      TTF_CloseFont(_font_sdl_ptr);
  };

  int text_width(const std::string &text) {
    int width = 0;
    if (TTF_SizeText(_font_sdl_ptr, text.c_str(), &width, nullptr) != 0) {
      ttf_error("Could not get text size");
    }
    return width;
  };

  int text_shaded(SDLRenderer &renderer, const std::string &text,
                  SDLColor color, SDLColor bgcolor, int x, int y) {

    SDL_Surface *text_surface_ptr = TTF_RenderText_Shaded(
        _font_sdl_ptr, text.c_str(), _sdl_color(color), _sdl_color(bgcolor));
    if (text_surface_ptr == nullptr)
      ttf_error("Could not render text");

    SDLTexture text_texture = renderer.create_texture(text_surface_ptr);
    int width = text_surface_ptr->w;

    SDL_Rect target_rect{.x = x, .y = y, .w = width, .h = text_surface_ptr->h};

    renderer.copy_to(text_texture, target_rect);

    SDL_FreeSurface(text_surface_ptr);

    return width;
  }
};
