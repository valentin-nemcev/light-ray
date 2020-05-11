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
using Eigen::Vector3d;

using Pixel = Eigen::Matrix<unsigned int, 2, 1>;

Eigen::IOFormat VFmt(Eigen::StreamPrecision, 0, ", ", ";", "", "", "(", ")");

Vector3d directionUp(0, 0, 1);

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

static const RayBounce noBounce = {.distance = -1};

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
      return noBounce;
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

void randomlyRotate(Vector3d &v) {
  static std::default_random_engine e;
  static std::uniform_real_distribution<> randomScalar(0, 1);

  Vector3d randomVector(randomScalar(e), randomScalar(e), randomScalar(e));

  randomVector -= randomVector.dot(v) * v; // make it orthogonal to v

  v += randomVector;
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

  int intR() const { return static_cast<int>(255.0 * r); }

  int intG() const { return static_cast<int>(255.0 * g); }

  int intB() const { return static_cast<int>(255.0 * b); }
};

struct CameraPixel {
  const Pixel pixel;
  const Ray ray;
  PixelValue value;

  void render(Scene const &scene) {
    int total = 2;
    for (int i = 0; i < total; i++)
      value.add(trace(scene));
    value.average(total);
  }

  PixelValue trace(Scene const &scene) {
    RayBounce closestBounce = noBounce;
    for (auto &shape : scene) {
      RayBounce bounce = shape->intersect(ray);
      if (bounce.bounced() && (!closestBounce.bounced() ||
                               bounce.distance < closestBounce.distance)) {
        closestBounce = bounce;
      }
    }

    randomlyRotate(closestBounce.normal);

    PixelValue value;

    if (closestBounce.bounced()) {
      double hit = exp(-closestBounce.distance * 0.25);
      // double hit = pow(cos(closestBounce.distance * 20), 64);
      value.r = hit * (closestBounce.normal.x() / 2 + 0.5);
      value.g = hit * (closestBounce.normal.y() / 2 + 0.5);
      value.b = hit * (closestBounce.normal.z() / 2 + 0.5);
    } else {
      value.r = value.g = value.b = 0;
    }

    return value;
  }
};

using Pixels = std::vector<CameraPixel>;

class Camera {
private:
  CameraPixel indexToPixel(const unsigned int index) const {
    const Pixel pixel(index % imageSize.x(), index / imageSize.x());
    const double aspectRatio =
        static_cast<double>(imageSize.x()) / imageSize.y();
    const Vector2d pixelPos((pixel.x() + 0.5) / imageSize.x() - 0.5,
                            (pixel.y() + 0.5) / imageSize.y() / aspectRatio -
                                0.5);

    Vector3d direction = (target - pos).normalized();
    const Vector3d cameraRight = direction.cross(directionUp);
    const Vector3d cameraUp = cameraRight.cross(direction);
    Vector3d rayDirection =
        (direction + cameraRight * pixelPos.x() - cameraUp * pixelPos.y())
            .normalized();
    return {.pixel = pixel, .ray = {.origin = pos, .direction = rayDirection}};
  }

  Pixels allocatePixels() const {
    Pixels pixels;
    const Pixel::Scalar pixelCount = imageSize.x() * imageSize.y();
    pixels.reserve(pixelCount);
    for (int i = 0; i < pixelCount; i++) {
      pixels.push_back(indexToPixel(i));
    }
    return pixels;
  }

public:
  const Vector3d pos;
  const Vector3d target;
  const Pixel imageSize;
  Pixels pixels;

  Camera(Vector3d const initial_pos, Vector3d const initial_target,
         Pixel const initial_imageSize)
      : pos(initial_pos), target(initial_target), imageSize(initial_imageSize),
        pixels(allocatePixels()) {}
};

class Caster {
  std::span<CameraPixel> pixelSpan;
  const Scene &scene;

  std::thread thread;

  void render() {
    for (auto &pixel : pixelSpan) {
      pixel.render(scene);
    }
    isRendering = false;
  }

public:
  bool isRendering = false;

  Caster(std::span<CameraPixel> _pixelSpan, Scene const &_scene)
      : pixelSpan(_pixelSpan), scene(_scene) {}

  void startRendering() {
    isRendering = true;
    thread = std::thread(&Caster::render, this);
  }

  void waitUntilRendered() {
    if (thread.joinable())
      thread.join();
  }
};

class Display {
  SDL_Window *window = nullptr;
  SDL_Surface *screenSurface = nullptr;

  void sdlError(std::string message) {
    throw std::runtime_error(
        boost::str(format("%s: %s\n") % message % SDL_GetError()));
  }

  std::chrono::time_point<std::chrono::system_clock> measureStartTime;

public:
  bool isRunning = true;

  Display() {
    if (SDL_Init(SDL_INIT_VIDEO) > 0) {
      sdlError("Could not initialize SDL2");
    }

    window = SDL_CreateWindow("Light Ray", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
                              SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == nullptr) {
      sdlError("Could not create window");
    }

    screenSurface = SDL_GetWindowSurface(window);
  }

  ~Display() {
    if (window != nullptr)
      SDL_DestroyWindow(window);
    SDL_Quit();
  }

  void drawPixels(Pixels const &pixels) {
    for (auto &pixel : pixels) {
      if (!isRunning)
        break;

      const SDL_Rect rect = {.x = static_cast<int>(pixel.pixel.x()),
                             .y = static_cast<int>(pixel.pixel.y()),
                             .w = 1,
                             .h = 1};
      SDL_FillRect(screenSurface, &rect,
                   SDL_MapRGB(screenSurface->format, pixel.value.intR(),
                              pixel.value.intG(), pixel.value.intB()));
    }
  }

  void update() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      processEvent(event);
    }

    SDL_UpdateWindowSurface(window);
  }

  void processEvent(SDL_Event &event) {
    if (event.type == SDL_QUIT) {
      isRunning = false;
    }
  }

  void waitForEvent() {
    SDL_Event event;
    if (SDL_WaitEvent(&event)) {
      processEvent(event);
    }
  }

  void startMeasure() { measureStartTime = std::chrono::system_clock::now(); }

  void completeMeasure(std::string const message) {

    const auto duration = std::chrono::system_clock::now() - measureStartTime;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std::cout << message << ms << "ms" << std::endl;
  }
};

int main(int /*argc*/, char * /*args*/[]) {
  Display display;

  Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0),
                Pixel(SCREEN_WIDTH, SCREEN_HEIGHT));

  Scene scene;

  scene.reserve(4);

  scene.push_back(std::make_unique<Plane>(Vector3d(0, 0, 0), directionUp));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0.3, 0.3, 0.3), 0.3));
  scene.push_back(std::make_unique<Sphere>(Vector3d(0, -0.5, 0.5), 0.5));
  scene.push_back(std::make_unique<Sphere>(Vector3d(-0.6, 0, 0.6), 0.6));

  display.update();

  display.startMeasure();

  const auto threadCount = std::thread::hardware_concurrency();
  const auto pixelsPerCaster =
      std::div(static_cast<int>(camera.pixels.size()), threadCount);

  std::vector<Caster> casters;
  casters.reserve(threadCount);
  for (int i = 0; i < threadCount; i++) {
    const int start = pixelsPerCaster.quot * i;
    const int count =
        pixelsPerCaster.quot + (i == threadCount - 1 ? pixelsPerCaster.rem : 0);
    casters.emplace_back(
        std::span<CameraPixel>(camera.pixels).subspan(start, count),
        std::cref(scene));
    casters[i].startRendering();
  }

  while (display.isRunning) {
    bool isRendering = false;
    for (auto &caster : casters)
      isRendering = isRendering || caster.isRendering;
    display.drawPixels(camera.pixels);
    display.update();
    if (!isRendering)
      break;
  }

  for (auto &caster : casters)
    caster.waitUntilRendered();
  display.completeMeasure("Rendered in ");

  while (display.isRunning)
    display.waitForEvent();

  return 0;
}
