#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <boost/format.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <span>
#include <stdexcept>
#include <thread>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

// https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
// length / magnitude is .norm()
using boost::format;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;

Eigen::IOFormat v_fmt(Eigen::StreamPrecision, 0, ", ", ";", "", "", "(", ")");

Vector3d direction_up(0, 0, 1);

struct Ray {
  const Vector3d origin;
  const Vector3d direction;
};

struct RayBounce {
  double distance;
  Vector3d point;
  Vector3d normal;

  bool bounced() { return distance >= 0; }
};

static const RayBounce no_bounce = {.distance = -1};

class Object {
public:
  virtual RayBounce intersect(const Ray &ray) const = 0;
  virtual ~Object() = default;
};

class Sphere : public Object {
public:
  const Vector3d pos;
  const double radius;

  // (pos - point).norm() == radius

  Sphere(Vector3d initial_pos, double initial_radius)
      : pos(initial_pos), radius(initial_radius){};

  virtual RayBounce intersect(const Ray &ray) const {
    // (pos - (ray.origin + ray.direction * distance)).norm == radius

    const double discriminant = pow(ray.direction.dot(ray.origin - pos), 2) -
                                (ray.origin - pos).squaredNorm() +
                                pow(radius, 2);
    if (discriminant < 0) {
      return no_bounce;
    }

    double distance = -ray.direction.dot(ray.origin - pos) - sqrt(discriminant);

    Vector3d normal =
        (pos - (ray.origin + ray.direction * distance)).normalized();
    return {.distance = distance, .normal = normal};
  }
};

class Plane : public Object {
public:
  const Vector3d pos;
  const Vector3d normal;

  // normal.dot(pos - point) == 0

  Plane(Vector3d initial_pos, Vector3d initial_normal)
      : pos(initial_pos), normal(initial_normal){};

  virtual RayBounce intersect(const Ray &ray) const {

    // normal.dot(pos - (ray.origin + ray.direction * distance)) == 0
    // normal.dot(pos - ray.origin) - normal.dot(ray.direction) * distance == 0
    // normal.dot(pos - ray.origin) / normal.dot(ray.direction) = distance

    return {.distance =
                normal.dot(pos - ray.origin) / normal.dot(ray.direction),
            .normal = normal};
  }
};

using Scene = std::vector<std::unique_ptr<Object>>;

void randomly_rotate(Vector3d &v) {
  static std::default_random_engine e;
  static std::uniform_real_distribution<> random_scalar(0, 1);

  Vector3d random_vector(random_scalar(e), random_scalar(e), random_scalar(e));

  random_vector -= random_vector.dot(v) * v; // make it orthogonal to v

  v += random_vector;
  v.normalize();
}

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

  int int_r() const { return static_cast<int>(255.0 * r); }

  int int_g() const { return static_cast<int>(255.0 * g); }

  int int_b() const { return static_cast<int>(255.0 * b); }
};

struct CameraPixel {
  const Vector2i coord;
  const Ray ray;
  PixelValue value;

  void render(Scene const &scene) {
    int total = 2;
    for (int i = 0; i < total; i++)
      value.add(trace(scene));
    value.average(total);
  }

  PixelValue trace(Scene const &scene) {
    RayBounce closest_bounce = no_bounce;
    for (auto &shape : scene) {
      RayBounce bounce = shape->intersect(ray);
      if (bounce.bounced() && (!closest_bounce.bounced() ||
                               bounce.distance < closest_bounce.distance)) {
        closest_bounce = bounce;
      }
    }

    randomly_rotate(closest_bounce.normal);

    PixelValue value;

    if (closest_bounce.bounced()) {
      double hit = exp(-closest_bounce.distance * 0.25);
      // double hit = pow(cos(closestBounce.distance * 20), 64);
      value.r = hit * (closest_bounce.normal.x() / 2 + 0.5);
      value.g = hit * (closest_bounce.normal.y() / 2 + 0.5);
      value.b = hit * (closest_bounce.normal.z() / 2 + 0.5);
    } else {
      value.r = value.g = value.b = 0;
    }

    return value;
  }
};

using Pixels = std::vector<CameraPixel>;

class Camera {
private:
  CameraPixel index_to_pixel(const unsigned int index) const {
    const Vector2i coord(index % image_size.x(), index / image_size.x());
    const double aspect_ratio =
        static_cast<double>(image_size.x()) / image_size.y();
    const Vector2d pixel_pos((coord.x() + 0.5) / image_size.x() - 0.5,
                             (coord.y() + 0.5) / image_size.y() / aspect_ratio -
                                 0.5);

    Vector3d direction = (target - pos).normalized();
    const Vector3d camera_right = direction.cross(direction_up);
    const Vector3d camera_up = camera_right.cross(direction);
    Vector3d ray_direction =
        (direction + camera_right * pixel_pos.x() - camera_up * pixel_pos.y())
            .normalized();
    return {.coord = coord, .ray = {.origin = pos, .direction = ray_direction}};
  }

  Pixels allocate_pixels() const {
    Pixels pixels;
    const Vector2i::Scalar pixel_count = image_size.x() * image_size.y();
    pixels.reserve(pixel_count);
    for (int i = 0; i < pixel_count; i++) {
      pixels.push_back(index_to_pixel(i));
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
        image_size(initial_image_size), pixels(allocate_pixels()) {}
};

class Caster {
  std::span<CameraPixel> pixel_span;
  const Scene &scene;

  std::thread thread;

  void render() {
    for (auto &pixel : pixel_span) {
      pixel.render(scene);
    }
    is_rendering = false;
  }

public:
  bool is_rendering = false;

  Caster(std::span<CameraPixel> pixel_span, Scene const &scene)
      : pixel_span(pixel_span), scene(scene) {}

  void start_rendering() {
    is_rendering = true;
    thread = std::thread(&Caster::render, this);
  }

  void wait_until_rendered() {
    if (thread.joinable())
      thread.join();
  }
};

class Display {
  SDL_Window *window = nullptr;
  SDL_Surface *screen_surface = nullptr;

  void sdl_error(std::string message) {
    throw std::runtime_error(
        boost::str(format("%s: %s\n") % message % SDL_GetError()));
  }

  std::chrono::time_point<std::chrono::system_clock> measure_start_time;

public:
  bool is_running = true;

  Display() {
    if (SDL_Init(SDL_INIT_VIDEO) > 0) {
      sdl_error("Could not initialize SDL2");
    }

    window = SDL_CreateWindow("Light Ray", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
                              SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
      sdl_error("Could not create window");
    }

    screen_surface = SDL_GetWindowSurface(window);
  }

  ~Display() {
    if (window != nullptr)
      SDL_DestroyWindow(window);
    SDL_Quit();
  }

  void draw_pixels(Pixels const &pixels) {
    for (auto &pixel : pixels) {
      if (!is_running)
        break;

      const SDL_Rect rect = {.x = static_cast<int>(pixel.coord.x()),
                             .y = static_cast<int>(pixel.coord.y()),
                             .w = 1,
                             .h = 1};
      SDL_FillRect(screen_surface, &rect,
                   SDL_MapRGB(screen_surface->format, pixel.value.int_r(),
                              pixel.value.int_g(), pixel.value.int_b()));
    }
  }

  void update() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      process_event(event);
    }

    SDL_UpdateWindowSurface(window);
  }

  void process_event(SDL_Event &event) {
    if (event.type == SDL_QUIT) {
      is_running = false;
    }
  }

  void wait_for_event() {
    SDL_Event event;
    if (SDL_WaitEvent(&event)) {
      process_event(event);
    }
  }

  void start_measure() {
    measure_start_time = std::chrono::system_clock::now();
  }

  void complete_measure(std::string const message) {

    const auto duration = std::chrono::system_clock::now() - measure_start_time;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std::cout << message << ms << "ms" << std::endl;
  }
};

int main(int /*argc*/, char * /*args*/[]) {
  Display display;

  Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0),
                Vector2i(SCREEN_WIDTH, SCREEN_HEIGHT));

  Scene scene;

  scene.reserve(4);

  scene.push_back(std::make_unique<Plane>(Vector3d(0, 0, 0), direction_up));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0.3, 0.3, 0.3), 0.3));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0, -0.5, 0.5), 0.5));
  scene.push_back(std::make_unique<Sphere>(Vector3d(-0.6, 0, 0.6), 0.6));

  display.update();

  display.start_measure();

  const auto thread_count = std::thread::hardware_concurrency();
  const auto pixels_per_caster =
      std::div(static_cast<int>(camera.pixels.size()), thread_count);

  std::vector<Caster> casters;
  casters.reserve(thread_count);
  for (unsigned i = 0; i < thread_count; i++) {
    const int start = pixels_per_caster.quot * i;
    const int count = pixels_per_caster.quot +
                      (i == thread_count - 1 ? pixels_per_caster.rem : 0);
    casters.emplace_back(
        std::span<CameraPixel>(camera.pixels).subspan(start, count),
        std::cref(scene));
    casters[i].start_rendering();
  }

  while (display.is_running) {
    bool is_rendering = false;
    for (auto &caster : casters)
      is_rendering = is_rendering || caster.is_rendering;
    display.draw_pixels(camera.pixels);
    display.update();
    if (!is_rendering)
      break;
  }

  for (auto &caster : casters)
    caster.wait_until_rendered();
  display.complete_measure("Rendered in ");

  while (display.is_running)
    display.wait_for_event();

  return 0;
}
