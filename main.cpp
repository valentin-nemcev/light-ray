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
#include <ostream>
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
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;

Eigen::IOFormat v_fmt(Eigen::StreamPrecision, 0, ", ", ";", "", "", "(", ")");

// Coordinate system is right-handed, i.e.
// direction_forward.cross(direction_left) == direction_up;
static const Vector3d direction_forward(1, 0, 1);
static const Vector3d direction_left(0, 1, 0);
static const Vector3d direction_up(0, 0, 1);

constexpr auto pi = M_PI;
constexpr double deg_to_rad(const double deg) { return pi * (deg / 180.0); }

Vector3d reflect(Vector3d const &direction, Vector3d const &normal) {
  return direction - 2 * direction.dot(normal) * normal;
};

std::random_device random_device;
static thread_local std::mt19937_64 random_engine(random_device());

Vector3d randomly_rotated(const Vector3d &vector) {
  static thread_local std::uniform_real_distribution<> r_coord(-1, 1);
  static thread_local std::uniform_real_distribution<> r_coeff(0, 1);
  static thread_local std::uniform_real_distribution<> r_2pi(0, 2 * pi);
  static thread_local std::uniform_real_distribution<> r_half_pi(0, pi / 4);
  static thread_local std::uniform_real_distribution<> r_length(
      0, std::numeric_limits<double>::infinity());

  Vector3d relative_left(std::fabs(vector.dot(direction_up)) < 1
                             ? direction_up.cross(vector)
                             : direction_left);
  relative_left.normalize();
  const Vector3d relative_up(vector.cross(relative_left));

  const double alpha = r_2pi(random_engine);
  const double beta = r_half_pi(random_engine);

  return (std::sin(beta) * std::sin(alpha) * relative_up +
          std::sin(beta) * std::cos(alpha) * relative_left +
          std::cos(beta) * vector)
      .normalized();
}

Vector3d random_interpolated_vector(const Vector3d &from, const Vector3d &to) {
  static thread_local std::uniform_real_distribution<> t(0, 1);
  return Vector3d(std::lerp(from.x(), to.x(), t(random_engine)),
                  std::lerp(from.y(), to.y(), t(random_engine)),
                  std::lerp(from.z(), to.z(), t(random_engine)));
}

double random_coefficient() {
  static thread_local std::uniform_real_distribution<> t(0, 1);
  return t(random_engine);
}

struct Ray {
  Vector3d origin;
  Vector3d direction;

  [[nodiscard]] Vector3d advance(const double distance) const {
    return origin + direction * distance;
  }
};

struct RayTermination {
  double value;
};

using RayIntersection = std::variant<RayTermination, Ray>;

static const double no_intersection = -1;

class Shape {
public:
  [[nodiscard]] virtual double intersection_distance(const Ray &ray) const = 0;
  [[nodiscard]] virtual Vector3d normal_at(const Vector3d &point) const = 0;

  Shape() = default;
  Shape(const Shape &) = default;
  Shape(Shape &&) = default;
  Shape &operator=(const Shape &) = default;
  Shape &operator=(Shape &&) = default;
  virtual ~Shape() = default;
};

class Surface {
public:
  [[nodiscard]] virtual RayIntersection
  intersect_at(const Vector3d &point, const Vector3d &normal,
               const Vector3d &direction) const = 0;

  Surface() = default;
  Surface(const Surface &) = default;
  Surface(Surface &&) = default;
  Surface &operator=(const Surface &) = default;
  Surface &operator=(Surface &&) = default;
  virtual ~Surface() = default;
};

class Object {
public:
  const std::unique_ptr<const Shape> shape;
  const std::unique_ptr<const Surface> surface;

  Object(std::unique_ptr<const Shape> shape,
         std::unique_ptr<const Surface> surface)
      : shape(std::move(shape)), surface(std::move(surface)) {}
};

class Diffuse : public Surface {
public:
  const double blackness;

  Diffuse(double blackness) : blackness(blackness){};

  [[nodiscard]] RayIntersection
  intersect_at(const Vector3d &point, const Vector3d &normal,
               const Vector3d & /*direction*/) const override {

    const double beta =
        std::uniform_real_distribution<double>(0, pi)(random_engine);

    if (random_coefficient() > std::cos(beta))
      return RayTermination{.value = 0};

    Vector3d relative_left(std::fabs(normal.dot(direction_up)) < 1
                               ? direction_up.cross(normal)
                               : direction_left);
    relative_left.normalize();
    const Vector3d relative_up(normal.cross(relative_left));

    const double alpha =
        std::uniform_real_distribution<double>(0, 2 * pi)(random_engine);

    const auto reflected = (std::sin(beta) * std::sin(alpha) * relative_up +
                            std::sin(beta) * std::cos(alpha) * relative_left +
                            std::cos(beta) * normal)
                               .normalized();
    return Ray{.origin = point, .direction = reflected};
  };
};

class Reflective : public Surface {
public:
  const double blackness;

  Reflective(double blackness) : blackness(blackness){};

  [[nodiscard]] RayIntersection
  intersect_at(const Vector3d &point, const Vector3d &normal,
               const Vector3d &direction) const override {

    if (random_coefficient() > blackness)
      return RayTermination{.value = 0};

    Vector3d surface_normal = randomly_rotated(normal);
    return Ray{.origin = point,
               .direction = reflect(direction, surface_normal)};
  };
};

class Sphere : public Shape {
public:
  const Vector3d pos;
  const double radius;

  // (pos - point).norm() == radius

  Sphere(Vector3d pos, double radius) : pos(std::move(pos)), radius(radius){};

  [[nodiscard]] double intersection_distance(const Ray &ray) const override {
    // (pos - (ray.origin + ray.direction * distance)).norm == radius

    const double discriminant = pow(ray.direction.dot(ray.origin - pos), 2) -
                                (ray.origin - pos).squaredNorm() +
                                pow(radius, 2);
    if (discriminant < 0)
      return no_intersection;

    return -ray.direction.dot(ray.origin - pos) - sqrt(discriminant);
  }

  [[nodiscard]] Vector3d normal_at(const Vector3d &point) const override {
    return (point - pos).normalized();
  }
};

class Plane : public Shape {
public:
  const Vector3d pos;
  const Vector3d normal;

  // normal.dot(pos - point) == 0

  Plane(Vector3d pos, Vector3d normal)
      : pos(std::move(pos)), normal(std::move(normal)){};

  [[nodiscard]] double intersection_distance(const Ray &ray) const override {

    // normal.dot(pos - (ray.origin + ray.direction * distance)) == 0
    // normal.dot(pos - ray.origin) - normal.dot(ray.direction) * distance == 0
    // normal.dot(pos - ray.origin) / normal.dot(ray.direction) = distance

    return normal.dot(pos - ray.origin) / normal.dot(ray.direction);
  }

  [[nodiscard]] Vector3d normal_at(const Vector3d & /*point*/) const override {
    return normal;
  }
};

class SkyShape : public Shape {
public:
  [[nodiscard]] double
  intersection_distance(const Ray & /*ray*/) const override {
    return std::numeric_limits<double>::infinity();
  }

  [[nodiscard]] Vector3d normal_at(const Vector3d &direction) const override {
    return -direction;
  }
};

class SkySurface : public Surface {
public:
  const Vector3d sun_direction;
  const double sun_angular_size_cos;
  const double sun_brightness;
  const double sky_brightness;

  SkySurface(const Vector3d &sun_direction, const double sun_angular_size,
             const double sun_brightness, const double sky_brightness)
      : sun_direction(sun_direction.normalized()),
        sun_angular_size_cos(std::cos(sun_angular_size / 2)),
        sun_brightness(sun_brightness), sky_brightness(sky_brightness){};

  [[nodiscard]] RayIntersection
  intersect_at(const Vector3d & /* point */, const Vector3d & /*normal*/,
               const Vector3d &direction) const override {

    const bool into_sun = direction.dot(sun_direction) > sun_angular_size_cos;

    return RayTermination{.value = into_sun ? sun_brightness : sky_brightness};
  };
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

  [[nodiscard]] int int_r() const {
    return std::min(255, static_cast<int>(255.0 * r));
  }

  [[nodiscard]] int int_g() const {
    return std::min(255, static_cast<int>(255.0 * g));
  }

  [[nodiscard]] int int_b() const {
    return std::min(255, static_cast<int>(255.0 * b));
  }
};

struct CameraPixel {
  const Vector2i coord;
  const Vector3d origin;
  const Vector3d top_left;
  const Vector3d bottom_right;
  PixelValue value;
  bool empty = true;

  void render(SceneRef scene) {
    const int iterations = std::pow(2, 18);
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
  closest_intersecting_object(SceneRef scene, const Object *prev_object,
                              const Ray &ray) {
    double closest_distance = std::numeric_limits<double>::infinity();
    const Object *closest_object = nullptr;
    for (const auto &object : scene) {
      if (object.get() == prev_object)
        continue;
      const double distance = object->shape->intersection_distance(ray);
      if (distance > 0 && distance <= closest_distance) {
        closest_distance = distance;
        closest_object = object.get();
      }
    }
    return std::make_pair(closest_distance, closest_object);
  }

  [[nodiscard]] PixelValue trace(SceneRef scene) const {
    Ray ray = emit();
    RayTermination result{.value = -1};

    const Object *prev_object = nullptr;
    constexpr int max_bounces = 16;
    for (int bounce = 0; bounce < max_bounces; bounce++) {
      auto [distance, object] =
          closest_intersecting_object(scene, prev_object, ray);

      if (object == nullptr) {
        log_message((boost::format("Nothing hit") % distance).str());
        break;
      }

      prev_object = object;

      const Vector3d point = ray.advance(distance);
      const Vector3d normal = object->shape->normal_at(point);
      const RayIntersection this_intersection =
          object->surface->intersect_at(point, normal, ray.direction);

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
        log_message(
            (boost::format("Max bounces reached, last distance: %e") % distance)
                .str());
    }

    PixelValue value{};

    if (result.value >= 0)
      value.r = value.g = value.b = result.value;
    else {
      value.r = result.value;
      value.g = value.b = 0;
    }

    return value;
  }

  void log_message(const std::string &message) const {
    std::cout << boost::str(boost::format("%4dx%4d: %s\n") % coord.x() %
                            coord.y() % message);
  }
};

using Pixels = std::vector<CameraPixel>;

class Camera {
private:
  [[nodiscard]] CameraPixel _index_to_pixel(const unsigned int index) const {
    const Vector2i coord(index % image_size.x(), index / image_size.x());
    const Vector2d pixel_pos_top_left(
        (coord.x() + 0.0) / image_size.x() - 0.5,
        ((coord.y() + 0.0) / image_size.y() - 0.5) / aspect_ratio);
    const Vector2d pixel_pos_bottom_right(
        (coord.x() + 1.0) / image_size.x() - 0.5,
        ((coord.y() + 1.0) / image_size.y() - 0.5) / aspect_ratio);

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
  const double aspect_ratio;
  const Vector3d direction;
  const Vector3d camera_right, camera_up;
  Pixels pixels;

  Camera(Vector3d pos, Vector3d target, Vector2i image_size)
      : pos(std::move(pos)), target(std::move(target)),
        image_size(std::move(image_size)),
        aspect_ratio(static_cast<double>(image_size.x()) / image_size.y()),
        direction((target - pos).normalized()),

        camera_right(std::fabs(direction.dot(direction_up)) < 1
                         ? direction.cross(direction_up)
                         : -direction_left),
        camera_up(camera_right.cross(direction)),

        pixels(_allocate_pixels()) {}
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

  void complete_measure(std::string const &message) {

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

  std::cout << boost::format("Image dimensions: %dx%d\n") %
                   screen_dimensions.width % screen_dimensions.height;

  // Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0.5),
  //               Vector2i(screen_dimensions.width, screen_dimensions.height));

  Camera camera(Vector3d(2, 3, 4), Vector3d(0, 0, 0.5),
                Vector2i(screen_dimensions.width, screen_dimensions.height));

  std::cout << boost::format("Allocated camera\n");
  Scene scene;

  scene.push_back(std::make_unique<Object>(
      std::make_unique<Plane>(Vector3d(0, 0, 0), direction_up),
      std::make_unique<Diffuse>(0.25)));
  scene.push_back(std::make_unique<Object>(
      std::make_unique<Sphere>(Vector3d(0.3, 0.3, 0.3), 0.3),
      std::make_unique<Diffuse>(0.5)));
  scene.push_back(std::make_unique<Object>(
      std::make_unique<Sphere>(Vector3d(0, -0.5, 0.4), 0.4),
      std::make_unique<Diffuse>(0.5)));
  scene.push_back(std::make_unique<Object>(
      std::make_unique<Sphere>(Vector3d(-0.8, -0, 0.6), 0.6),
      std::make_unique<Diffuse>(0.5)));
  // scene.push_back(std::make_unique<Sphere>(Vector3d(0, -0.5, 0.5), 0.5,
  // 0.25)); scene.push_back(std::make_unique<Sphere>(Vector3d(-0.6, 0, 0.6),
  // 0.6, 0.75)); scene.push_back(std::make_unique<Sphere>(Vector3d(0, 0, 0),
  // 0.75, 0.75));

  scene.push_back(std::make_unique<Object>(
      std::make_unique<SkyShape>(),
      std::make_unique<SkySurface>(Vector3d(-4, 6, 4), deg_to_rad(15 /*0.53*/),
                                   25, 0.25)));

  std::cout << boost::format("Allocated scene\n");
  display.draw_background();
  display.update();

  std::cout << boost::format("Window ready\n");
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
