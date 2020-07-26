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

struct CameraPixel {
  double v = 0;
  unsigned long iterations = 0;

  void add(double value) {
    v += value;
    iterations++;
  }

  [[nodiscard]] bool empty() const { return iterations == 0U; }

  [[nodiscard]] double average_v() const { return v / (double)iterations; }

  [[nodiscard]] int int_r() const {
    return std::min(255, static_cast<int>(255.0 * average_v()));
  }

  [[nodiscard]] int int_g() const {
    return std::min(255, static_cast<int>(255.0 * average_v()));
  }

  [[nodiscard]] int int_b() const {
    return std::min(255, static_cast<int>(255.0 * average_v()));
  }

  static Vector2i index_to_coord(const int index, const int width) {
    return Vector2i(index % width, index / width);
  }
};

using Pixels = std::vector<CameraPixel>;

class Camera {
private:
  [[nodiscard]] Pixels _allocate_pixels() const {
    const Vector2i::Scalar pixel_count = image_size.x() * image_size.y();
    Pixels pixels(pixel_count);
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

struct PixelChunk {
  int begin_index;
  int end_index;
};

class Caster {
  Pixels &_pixels;
  PixelChunk _pixel_chunk;
  SceneRef _scene;
  const Camera &_camera;

  std::thread _thread;
  std::atomic<bool> _is_rendering = false;

  [[nodiscard]] static Ray _emit(const Camera &camera, const int pixel_index) {
    const Vector2i coord =
        CameraPixel::index_to_coord(pixel_index, camera.image_size.x());
    const Vector2d pixel_pos(
        (coord.x() + random_coefficient()) / camera.image_size.x() - 0.5,
        ((coord.y() + random_coefficient()) / camera.image_size.y() - 0.5) /
            camera.aspect_ratio);

    Vector3d direction =
        (camera.direction + camera.camera_right * pixel_pos.x() -
         camera.camera_up * pixel_pos.y())
            .normalized();
    return {.origin = camera.pos, .direction = direction};
  };

  static std::pair<double, const Object *>
  _closest_intersecting_object(SceneRef scene, const Object *prev_object,
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

  [[nodiscard]] static double _trace(SceneRef scene, const Camera &camera,
                                     const int pixel_index) {
    Ray ray = _emit(camera, pixel_index);
    RayTermination result{.value = -1};

    const Object *prev_object = nullptr;
    constexpr int max_bounces = 16;
    for (int bounce = 0; bounce < max_bounces; bounce++) {
      auto [distance, object] =
          _closest_intersecting_object(scene, prev_object, ray);

      if (object == nullptr) {
        _log_message(pixel_index,
                     (boost::format("Nothing hit") % distance).str());
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
        _log_message(
            pixel_index,
            (boost::format("Max bounces reached, last distance: %e") % distance)
                .str());
    }

    if (result.value < 0)
      _log_message(
          pixel_index,
          (boost::format("Negative pixel value: %e") % result.value).str());

    return result.value;
  }

  static void _log_message(const int pixel_index, const std::string &message) {
    std::cout << boost::str(boost::format("%8d: %s\n") % pixel_index % message);
  }

  static void _render_pixel(SceneRef scene, const Camera &camera,
                            Pixels &pixels, const int pixel_index) {
    const int iterations = std::pow(2, 12);
    auto &pixel = pixels[pixel_index];
    for (int i = 0; i < iterations; i++)
      pixel.add(_trace(scene, camera, pixel_index));
  }

  void _render() {
    for (int pixel_index = _pixel_chunk.begin_index;
         pixel_index < _pixel_chunk.end_index; pixel_index++) {
      _render_pixel(_scene, _camera, _pixels, pixel_index);
      if (!_is_rendering)
        break;
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

  Caster(Pixels &pixels, PixelChunk pixel_chunk, SceneRef scene,
         const Camera &camera)
      : _pixels(pixels), _pixel_chunk(pixel_chunk), _scene(scene),
        _camera(camera) {}

  void start_rendering() {
    _is_rendering = true;
    _thread = std::thread(&Caster::_render, this);
  }

  void stop_rendering() {
    _is_rendering = false;
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

    int width = 0;
    int height = 0;
    SDL_GetRendererOutputSize(_renderer, &width, &height);

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

int main(int /*argc*/, char * /*args*/[]) {
  Display display;

  auto screen_dimensions = display.screen_dimensions();

  std::cout << boost::format("Image dimensions: %dx%d\n") %
                   screen_dimensions.width % screen_dimensions.height;

  // Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0.5),
  //               Vector2i(screen_dimensions.width, screen_dimensions.height));

  display.start_measure();
  Camera camera(Vector3d(2, 3, 4), Vector3d(0, 0, 0.5),
                Vector2i(screen_dimensions.width, screen_dimensions.height));

  display.complete_measure("Allocated camera in ");
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
    PixelChunk chunk = {.begin_index = static_cast<int>(start),
                        .end_index = static_cast<int>(start + count)};
    auto caster = std::make_unique<Caster>(std::ref(camera.pixels), chunk,
                                           std::cref(scene), std::cref(camera));
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
    caster->stop_rendering();
  display.complete_measure("Rendered in ");

  while (display.is_running())
    display.wait_for_event();

  return 0;
}
