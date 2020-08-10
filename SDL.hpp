#pragma once

#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <SDL_ttf/SDL_ttf.h>
#include <boost/format.hpp>
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

class SDLTexture {
  SDL_Texture *_texture;

public:
  SDLTexture(const SDLTexture &) = delete;
  SDLTexture(SDLTexture &&) = delete;
  SDLTexture &operator=(const SDLTexture &) = delete;
  SDLTexture &operator=(SDLTexture &&) = delete;

  SDLTexture(const SDLRenderer &renderer, int width, int height, int access,
             SDL_BlendMode blend_mode);
  SDLTexture(const SDLRenderer &renderer, SDL_Surface *surface,
             SDL_BlendMode blend_mode);

  ~SDLTexture();
  SDL_Texture *sdl_ptr() { return _texture; }

  SDLTextureLock lock();
};

class SDLWindow {
  SDL_Window *_window;
  int _pixel_width = 0;
  int _pixel_height = 0;
  int _display_scale = 0;

public:
  [[nodiscard]] int display_scale() const { return _display_scale; }

  SDLWindow(const SDLWindow &) = delete;
  SDLWindow(SDLWindow &&) = delete;
  SDLWindow &operator=(const SDLWindow &) = delete;
  SDLWindow &operator=(SDLWindow &&) = delete;

  SDLWindow(const int display_width, const int display_height,
            const int display_index) {
    _window = SDL_CreateWindow(
        "Light Ray", SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index),
        SDL_WINDOWPOS_UNDEFINED_DISPLAY(display_index), display_width,
        display_height, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (_window == nullptr)
      sdl_error("Could not create window");

    SDL_GL_GetDrawableSize(_window, &_pixel_width, &_pixel_height);
    _display_scale = _pixel_width / display_width;
  }

  ~SDLWindow() {
    if (_window != nullptr)
      SDL_DestroyWindow(_window);
  }

  [[nodiscard]] SDL_Window *sdl_ptr() const { return _window; }

  SDLRenderer create_renderer();
  SDLTextureRenderer create_texture_renderer(SDLTexture &texture);
};

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
SDLWindow SDL::create_window(const int window_display_width,
                             const int window_display_height,
                             const int display_index) {
  return SDLWindow(window_display_width, window_display_height, display_index);
}

using SDLPixelFormat = Uint32;

class SDLRenderer {
  SDL_Renderer *_renderer;
  SDLPixelFormat _pixel_format;

public:
  SDLRenderer(const SDLRenderer &) = delete;
  SDLRenderer(SDLRenderer &&) = delete;
  SDLRenderer &operator=(const SDLRenderer &) = delete;
  SDLRenderer &operator=(SDLRenderer &&) = delete;

  SDLRenderer(const SDLWindow &window) {
    _renderer = SDL_CreateRenderer(window.sdl_ptr(), -1,
                                   SDL_RENDERER_ACCELERATED |
                                       SDL_RENDERER_PRESENTVSYNC);
    if (_renderer == nullptr)
      sdl_error("Could not create renderer");

    _pixel_format = SDL_GetWindowPixelFormat(window.sdl_ptr());
  }

  ~SDLRenderer() {
    if (_renderer != nullptr)
      SDL_DestroyRenderer(_renderer);
  }

  [[nodiscard]] SDL_Renderer *sdl_ptr() const { return _renderer; }

  [[nodiscard]] SDLPixelFormat pixel_format() const { return _pixel_format; }

  [[nodiscard]] SDLSize output_size() const {
    SDLSize size;
    SDL_GetRendererOutputSize(_renderer, &size.width, &size.height);
    return size;
  }

  void to_texture(SDLTexture &texture,
                  const std::function<void(SDLRenderer &)> &callback) {

    if (SDL_SetRenderTarget(sdl_ptr(), texture.sdl_ptr()) != 0)
      sdl_error("Could not set render target to texture");
    callback(*this);
    if (SDL_SetRenderTarget(sdl_ptr(), nullptr) != 0)
      sdl_error("Could not set render target to window");
  }

  SDLTexture create_texture(int width, int height, SDL_TextureAccess access,
                            SDL_BlendMode blend_mode = SDL_BLENDMODE_NONE);
  SDLTexture create_texture(SDL_Surface *surface,
                            SDL_BlendMode blend_mode = SDL_BLENDMODE_NONE);

  void fill_rect(const SDL_Rect &rect, SDLColor color) {
    SDL_SetRenderDrawColor(_renderer, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(_renderer, &rect);
  }

  void copy_to(SDLTexture &texture, SDL_Rect target_rect) {
    if (SDL_RenderCopy(_renderer, texture.sdl_ptr(), nullptr, &target_rect) !=
        0)
      sdl_error("Could not copy background texture");
  }

  void present() { SDL_RenderPresent(_renderer); }
};

SDLRenderer SDLWindow::create_renderer() { return SDLRenderer(*this); }

SDLTexture::SDLTexture(const SDLRenderer &renderer, int width, int height,
                       int access, SDL_BlendMode blend_mode) {
  _texture = SDL_CreateTexture(renderer.sdl_ptr(), renderer.pixel_format(),
                               access, width, height);
  if (_texture == nullptr)
    sdl_error("Could not create texture");

  if (SDL_SetTextureBlendMode(_texture, blend_mode) != 0)
    sdl_error("Could not set screen texture blend mode");
}

SDLTexture::SDLTexture(const SDLRenderer &renderer, SDL_Surface *surface,
                       SDL_BlendMode blend_mode) {

  _texture = SDL_CreateTextureFromSurface(renderer.sdl_ptr(), surface);
  if (_texture == nullptr)
    sdl_error("Could not create texture");

  if (SDL_SetTextureBlendMode(_texture, blend_mode) != 0)
    sdl_error("Could not set screen texture blend mode");
}

SDLTexture::~SDLTexture() {
  if (_texture != nullptr)
    SDL_DestroyTexture(_texture);
}
SDLTexture SDLRenderer::create_texture(int width, int height,
                                       SDL_TextureAccess access,
                                       SDL_BlendMode blend_mode) {
  return SDLTexture(*this, width, height, access, blend_mode);
}
SDLTexture SDLRenderer::create_texture(SDL_Surface *surface,
                                       SDL_BlendMode blend_mode) {
  return SDLTexture(*this, surface, blend_mode);
}

class SDLTextureLock {
  SDL_Texture *_texture;
  SDL_PixelFormat *_pixel_format = nullptr;
  std::span<Uint32> _texture_span;
  int _width = 0;
  int _height = 0;

public:
  SDLTextureLock(const SDLTextureLock &) = delete;
  SDLTextureLock(SDLTextureLock &&) = delete;
  SDLTextureLock &operator=(const SDLTextureLock &) = delete;
  SDLTextureLock &operator=(SDLTextureLock &&) = delete;

  SDLTextureLock(SDLTexture &texture) : _texture(texture.sdl_ptr()) {

    Uint32 format_tag = 0;

    int access = 0;
    SDL_QueryTexture(_texture, &format_tag, &access, &_width, &_height);

    _pixel_format = SDL_AllocFormat(format_tag);
    if (_pixel_format == nullptr)
      sdl_error("Could not allocate pixel format");

    void *texture_pixels = nullptr;
    int byte_pitch = 0;
    if (SDL_LockTexture(_texture, nullptr, &texture_pixels, &byte_pitch) != 0)
      sdl_error("Could not lock screen texture for updating");

    _texture_span =
        std::span(static_cast<Uint32 *>(texture_pixels), _width * _height);
  }

  ~SDLTextureLock() {
    SDL_UnlockTexture(_texture);
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

SDLTextureLock SDLTexture::lock() { return SDLTextureLock(*this); };

class SDLFont {
  TTF_Font *_font = nullptr;

  [[nodiscard]] static SDL_Color _sdl_color(const SDLColor color) {
    return {.r = color.r, .g = color.g, .b = color.b, .a = color.a};
  }

public:
  SDLFont(const SDLFont &) = delete;
  SDLFont(SDLFont &&) = delete;
  SDLFont &operator=(const SDLFont &) = delete;
  SDLFont &operator=(SDLFont &&) = delete;

  SDLFont(const std::string &font_file, int size) {

    _font = TTF_OpenFont(font_file.c_str(), size);
    if (_font == nullptr)
      ttf_error("Could not load font");
  };

  ~SDLFont() {

    if (_font != nullptr)
      TTF_CloseFont(_font);
  };
  TTF_Font *sdl_ptr() { return _font; }

  void text_shaded(SDLRenderer &renderer, const std::string &text,
                   SDLColor color, SDLColor bgcolor, int x, int y) {

    SDL_Surface *text_surface = TTF_RenderText_Shaded(
        _font, text.c_str(), _sdl_color(color), _sdl_color(bgcolor));
    if (text_surface == nullptr)
      ttf_error("Could not render text");

    SDLTexture text_texture = renderer.create_texture(text_surface);

    SDL_Rect target_rect{
        .x = x, .y = y, .w = text_surface->w, .h = text_surface->h};

    renderer.copy_to(text_texture, target_rect);

    SDL_FreeSurface(text_surface);
  }
};
