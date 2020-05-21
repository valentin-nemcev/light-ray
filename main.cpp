#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <boost/format.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>

// default constructors reference
// https://en.cppreference.com/w/cpp/language/rule_of_three

constexpr unsigned window_width = 640;
constexpr unsigned window_height = 480;

// https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
// length / magnitude is .norm()
using boost::format;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;

Eigen::IOFormat v_fmt(Eigen::StreamPrecision, 0, ", ", ";", "", "", "(", ")");

static const Vector3d direction_up(0, 0, 1);

constexpr auto pi = M_PI;
constexpr double deg_to_rad(const double deg) { return pi * (deg / 180.0); }

Vector3d reflect(Vector3d const direction, Vector3d const normal) {
  return direction - 2 * direction.dot(normal) * normal;
};

std::random_device random_device;
static thread_local std::default_random_engine random_engine(random_device());

void randomly_rotate(Vector3d &v) {
  static std::uniform_real_distribution<> random_scalar(0, 1);

  Vector3d random_vector(random_scalar(random_engine),
                         random_scalar(random_engine),
                         random_scalar(random_engine));

  random_vector -= random_vector.dot(v) * v; // make it orthogonal to v

  v += random_vector;
  v.normalize();
}

Vector3d randomly_rotated(Vector3d direction) {
  randomly_rotate(direction);
  return direction;
}

Vector3d random_interpolated_vector(const Vector3d from, const Vector3d to) {
  static std::uniform_real_distribution<> t(0, 1);
  return Vector3d(std::lerp(from.x(), to.x(), t(random_engine)),
                  std::lerp(from.y(), to.y(), t(random_engine)),
                  std::lerp(from.z(), to.z(), t(random_engine)));
}

double random_coefficient() {
  static std::uniform_real_distribution<> t(0, 1);
  return t(random_engine);
}

struct Ray {
  Vector3d origin;
  Vector3d direction;
};

struct RayTermination {
  double value;
};

using RayIntersection = std::variant<RayTermination, Ray>;

static const double no_intersection = -1;

class Object {
public:
  [[nodiscard]] virtual double intersection_distance(const Ray &ray) const = 0;
  [[nodiscard]] virtual RayIntersection intersect_at(const Ray &ray,
                                                     double distance) const = 0;

  Object() = default;
  Object(const Object &) = default;
  Object(Object &&) = default;
  Object &operator=(const Object &) = default;
  Object &operator=(Object &&) = default;
  virtual ~Object() = default;
};

class Sphere : public Object {
public:
  const Vector3d pos;
  const double radius;
  const double blackness;

  // (pos - point).norm() == radius

  Sphere(Vector3d initial_pos, double initial_radius, double blackness)
      : pos(std::move(initial_pos)), radius(initial_radius),
        blackness(blackness){};

  [[nodiscard]] double intersection_distance(const Ray &ray) const override {
    // (pos - (ray.origin + ray.direction * distance)).norm == radius

    const double discriminant = pow(ray.direction.dot(ray.origin - pos), 2) -
                                (ray.origin - pos).squaredNorm() +
                                pow(radius, 2);
    if (discriminant < 0) {
      return no_intersection;
    }
    return -ray.direction.dot(ray.origin - pos) - sqrt(discriminant);
  }

  [[nodiscard]] RayIntersection
  intersect_at(const Ray &ray, const double distance) const override {
    if (random_coefficient() > blackness)
      return RayTermination{.value = 0};

    Vector3d origin = ray.origin + ray.direction * distance;
    Vector3d normal = (pos - origin).normalized();
    randomly_rotate(normal);
    return Ray{.origin = origin, .direction = reflect(ray.direction, normal)};
  }
};

class Plane : public Object {
public:
  const Vector3d pos;
  const Vector3d normal;

  // normal.dot(pos - point) == 0

  Plane(Vector3d initial_pos, Vector3d initial_normal)
      : pos(std::move(initial_pos)), normal(std::move(initial_normal)){};

  [[nodiscard]] double intersection_distance(const Ray &ray) const override {

    // normal.dot(pos - (ray.origin + ray.direction * distance)) == 0
    // normal.dot(pos - ray.origin) - normal.dot(ray.direction) * distance == 0
    // normal.dot(pos - ray.origin) / normal.dot(ray.direction) = distance

    return normal.dot(pos - ray.origin) / normal.dot(ray.direction);
  }

  [[nodiscard]] RayIntersection
  intersect_at(const Ray &ray, const double distance) const override {
    if (random_coefficient() > 0.75)
      return RayTermination{.value = 0};

    Vector3d origin = ray.origin + ray.direction * distance;
    return Ray{.origin = origin,
               .direction = reflect(ray.direction, randomly_rotated(normal))};
  }
};

class Sky : public Object {
public:
  const Vector3d sun_direction;
  const double sun_angular_size_cos;

  Sky(const Vector3d sun_direction, const double sun_angular_size)
      : sun_direction(sun_direction.normalized()),
        sun_angular_size_cos(std::cos(sun_angular_size)){};

  [[nodiscard]] double
  intersection_distance(const Ray & /*ray*/) const override {
    return std::numeric_limits<double>::infinity();
  }

  [[nodiscard]] RayIntersection
  intersect_at(const Ray &ray, double /*distance*/) const override {

    const bool into_sun =
        ray.direction.dot(sun_direction) > sun_angular_size_cos;

    return RayTermination{.value = into_sun ? 1.0 : 0.25};
  }
};

using Scene = std::vector<std::unique_ptr<Object>>;
using SceneRef = const std::vector<std::unique_ptr<Object>> &;

struct PixelValue {
  double r;
  double g;
  double b;

  void add(PixelValue const &other) {
    r += other.r;
    g += other.g;
    b += other.b;
  }

  void average(int count) {
    r /= count;
    g /= count;
    b /= count;
  }

  [[nodiscard]] int int_r() const { return static_cast<int>(255.0 * r); }

  [[nodiscard]] int int_g() const { return static_cast<int>(255.0 * g); }

  [[nodiscard]] int int_b() const { return static_cast<int>(255.0 * b); }
};

struct CameraPixel {
  const Vector2i coord;
  const Vector3d origin;
  const Vector3d top_left;
  const Vector3d bottom_right;
  PixelValue value;
  bool empty = true;

  void render(SceneRef scene) {
    constexpr int iterations = 16;
    empty = false;
    for (int i = 0; i < iterations; i++)
      value.add(trace(scene));
    value.average(iterations);
  }

  [[nodiscard]] Ray emit() const {
    return {
        .origin = origin,
        .direction =
            random_interpolated_vector(top_left, bottom_right).normalized()};
  };

  static std::pair<double, const Object *>
  closest_intersecting_object(SceneRef scene, const Ray &ray) {
    double closest_distance = std::numeric_limits<double>::infinity();
    const Object *closest_object = nullptr;
    for (const auto &object : scene) {
      const double distance = object->intersection_distance(ray);
      if (distance >= 0 && distance <= closest_distance) {
        closest_distance = distance;
        closest_object = object.get();
      }
    }
    return std::make_pair(closest_distance, closest_object);
  }

  [[nodiscard]] PixelValue trace(SceneRef scene) const {
    Ray ray = emit();
    RayTermination result{.value = 0};

    constexpr int max_bounces = 64;
    for (int bounce = 0; bounce < max_bounces; bounce++) {
      auto [distance, object] = closest_intersecting_object(scene, ray);

      if (object == nullptr) {
        std::cout << format("Nothing hit\n");
        break;
      }

      const RayIntersection this_intersection =
          object->intersect_at(ray, distance);

      if (const auto *termination =
              std::get_if<RayTermination>(&this_intersection)) {
        result = *termination;
        break;
      }
      if (const auto *const bounced_ray =
              std::get_if<Ray>(&this_intersection)) {
        ray = *bounced_ray;
      }
      if (bounce == max_bounces - 1)
        std::cout << format("Max bounces reached\n");
    }

    PixelValue value{};

    value.r = value.g = value.b = result.value;

    // if (closest_intersection.intersected()) {
    //   double hit = exp(-closest_intersection.distance * 0.25);
    //   value.r = hit * (closest_intersection.normal.x() / 2 + 0.5);
    //   value.g = hit * (closest_intersection.normal.y() / 2 + 0.5);
    //   value.b = hit * (closest_intersection.normal.z() / 2 + 0.5);
    // } else {
    //   value.r = value.g = value.b = 0;
    // }

    return value;
  }
};

using Pixels = std::vector<CameraPixel>;

class Camera {
private:
  [[nodiscard]] CameraPixel _index_to_pixel(const unsigned int index) const {
    const Vector2i coord(index % image_size.x(), index / image_size.x());
    const double aspect_ratio =
        static_cast<double>(image_size.x()) / image_size.y();
    const Vector2d pixel_pos_top_left(
        (coord.x() + 0.0) / image_size.x() - 0.5,
        (coord.y() + 0.0) / image_size.y() / aspect_ratio - 0.5);
    const Vector2d pixel_pos_bottom_right(
        (coord.x() + 1.0) / image_size.x() - 0.5,
        (coord.y() + 1.0) / image_size.y() / aspect_ratio - 0.5);

    Vector3d direction = (target - pos).normalized();
    const Vector3d camera_right = direction.cross(direction_up);
    const Vector3d camera_up = camera_right.cross(direction);
    Vector3d ray_direction_top_left =
        (direction + camera_right * pixel_pos_top_left.x() -
         camera_up * pixel_pos_bottom_right.y())
            .normalized();
    Vector3d ray_direction_bottom_right =
        (direction + camera_right * pixel_pos_bottom_right.x() -
         camera_up * pixel_pos_bottom_right.y())
            .normalized();
    return {.coord = coord,
            .origin = pos,
            .top_left = ray_direction_top_left,
            .bottom_right = ray_direction_bottom_right};
  }

  [[nodiscard]] Pixels _allocate_pixels() const {
    Pixels pixels;
    const Vector2i::Scalar pixel_count = image_size.x() * image_size.y();
    pixels.reserve(pixel_count);
    for (int i = 0; i < pixel_count; i++) {
      pixels.push_back(_index_to_pixel(i));
    }
    return pixels;
  }

public:
  const Vector3d pos;
  const Vector3d target;
  const Vector2i image_size;
  Pixels pixels;

  Camera(Vector3d const initial_pos, Vector3d const initial_target,
         Vector2i const initial_image_size)
      : pos(initial_pos), target(initial_target),
        image_size(initial_image_size), pixels(_allocate_pixels()) {}
};

class Caster {
  const std::span<CameraPixel> _pixel_span;
  SceneRef _scene;

  std::thread _thread;
  bool _is_rendering = false;

  void _render() {
    for (auto &pixel : _pixel_span) {
      pixel.render(_scene);
    }
    _is_rendering = false;
  }

public:
  [[nodiscard]] bool is_rendering() const { return _is_rendering; }

  Caster() = delete;
  Caster(const Caster &) = delete;
  Caster(Caster &&) = delete;
  Caster &operator=(const Caster &) = delete;
  Caster &operator=(Caster &&) = delete;
  ~Caster() = default;

  Caster(const std::span<CameraPixel> pixel_span, SceneRef scene)
      : _pixel_span(pixel_span), _scene(scene) {}

  void start_rendering() {
    _is_rendering = true;
    _thread = std::thread(&Caster::_render, this);
  }

  void wait_until_rendered() {
    if (_thread.joinable())
      _thread.join();
  }
};

class Display {
  SDL_Window *_window = nullptr;
  SDL_Renderer *_renderer = nullptr;

  static void _sdl_error(std::string message) {
    throw std::runtime_error(
        boost::str(format("%s: %s\n") % message % SDL_GetError()));
  }

  std::chrono::time_point<std::chrono::system_clock> _measure_start_time;

  bool _is_running = true;

public:
  Display(const Display &) = delete;
  Display(Display &&) = delete;
  Display &operator=(const Display &) = delete;
  Display &operator=(Display &&) = delete;

  [[nodiscard]] bool is_running() const { return _is_running; }

  Display() {
    if (SDL_Init(SDL_INIT_VIDEO) > 0)
      _sdl_error("Could not initialize SDL2");

    _window =
        SDL_CreateWindow("Light Ray", SDL_WINDOWPOS_UNDEFINED,
                         SDL_WINDOWPOS_UNDEFINED, window_width, window_height,
                         SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (_window == nullptr)
      _sdl_error("Could not create window");

    _renderer = SDL_CreateRenderer(
        _window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (_renderer == nullptr)
      _sdl_error("Could not create renderer");
  }

  ~Display() {
    if (_renderer != nullptr)
      SDL_DestroyRenderer(_renderer);
    if (_window != nullptr)
      SDL_DestroyWindow(_window);
    SDL_Quit();
  }

  void draw_background() {
    int width = 0;
    int height = 0;
    constexpr int square_size = 16;
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
  }

  void draw_pixels(Pixels const &pixels) {
    draw_background();
    for (const auto &pixel : pixels) {
      if (!is_running())
        break;

      if (pixel.empty)
        continue;
      SDL_SetRenderDrawColor(_renderer, pixel.value.int_r(),
                             pixel.value.int_g(), pixel.value.int_b(),
                             SDL_ALPHA_OPAQUE);
      SDL_RenderDrawPoint(_renderer, static_cast<int>(pixel.coord.x()),
                          static_cast<int>(pixel.coord.y()));
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

  void complete_measure(std::string const message) {

    const auto duration =
        std::chrono::system_clock::now() - _measure_start_time;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std::cout << message << ms << "ms" << std::endl;
  }
};

int main(int /*argc*/, char * /*args*/[]) {
  Display display;

  auto screen_dimensions = display.screen_dimensions();

  std::cout << format("Image dimensions: %dx%d\n") % screen_dimensions.width %
                   screen_dimensions.height;

  Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0),
                Vector2i(screen_dimensions.width, screen_dimensions.height));

  std::cout << format("Allocated camera\n");
  Scene scene;

  scene.reserve(4);

  scene.push_back(std::make_unique<Plane>(Vector3d(0, 0, 0), direction_up));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0.3, 0.3, 0.3), 0.3, 0.5));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0, -0.5, 0.5), 0.5, 0.25));
  scene.push_back(std::make_unique<Sphere>(Vector3d(-0.6, 0, 0.6), 0.6, 0.75));
  scene.push_back(
      std::make_unique<Sky>(Vector3d(1, -1, 1), deg_to_rad(30 /*0.53*/)));

  std::cout << format("Allocated scene\n");
  display.draw_background();
  display.update();

  std::cout << format("Window ready\n");
  display.start_measure();

  const auto thread_count = std::thread::hardware_concurrency();
  const auto pixels_per_caster =
      std::div(static_cast<int>(camera.pixels.size()), thread_count);

  std::vector<std::unique_ptr<Caster>> casters;
  casters.reserve(thread_count);

  for (unsigned i = 0; i < thread_count; i++) {
    const unsigned start = pixels_per_caster.quot * i;
    const unsigned count = pixels_per_caster.quot +
                           (i == thread_count - 1 ? pixels_per_caster.rem : 0);
    auto caster = std::make_unique<Caster>(
        std::span<CameraPixel>(camera.pixels).subspan(start, count),
        std::cref(scene));
    caster->start_rendering();
    casters.push_back(std::move(caster));
  }

  while (display.is_running()) {
    bool is_rendering = false;
    for (auto &caster : casters)
      is_rendering = is_rendering || caster->is_rendering();
    display.draw_pixels(camera.pixels);
    display.update();
    if (!is_rendering)
      break;
  }

  for (auto &caster : casters)
    caster->wait_until_rendered();
  display.complete_measure("Rendered in ");

  while (display.is_running())
    display.wait_for_event();

  return 0;
}
