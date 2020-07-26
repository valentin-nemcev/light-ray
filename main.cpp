#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <boost/format.hpp>
#include <csignal>
#include <iostream>
#include <ostream>
#include <thread>

#include "display.hpp"
#include "renderer.hpp"

// default constructors reference
// https://en.cppreference.com/w/cpp/language/rule_of_three

constexpr unsigned window_width = 640;
constexpr unsigned window_height = 480;

struct PixelChunk {
  int begin_index;
  int end_index;
};

class Worker {
  Pixels &_pixels;
  PixelChunk _pixel_chunk;
  SceneRef _scene;
  const Camera &_camera;

  std::thread _thread;
  std::atomic<bool> _is_rendering = false;

  void _render() {
    for (int pixel_index = _pixel_chunk.begin_index;
         pixel_index < _pixel_chunk.end_index; pixel_index++) {

      auto &pixel = _pixels[pixel_index];
      Renderer::render_pixel(_scene, _camera, pixel, pixel_index);
      if (!_is_rendering)
        break;
    }
    _is_rendering = false;
  }

public:
  [[nodiscard]] bool is_rendering() const { return _is_rendering; }

  Worker() = delete;
  Worker(const Worker &) = delete;
  Worker(Worker &&) = delete;
  Worker &operator=(const Worker &) = delete;
  Worker &operator=(Worker &&) = delete;
  ~Worker() = default;

  Worker(Pixels &pixels, PixelChunk pixel_chunk, SceneRef scene,
         const Camera &camera)
      : _pixels(pixels), _pixel_chunk(pixel_chunk), _scene(scene),
        _camera(camera) {}

  void start_rendering() {
    _is_rendering = true;
    _thread = std::thread(&Worker::_render, this);
  }

  void stop_rendering() {
    _is_rendering = false;
    if (_thread.joinable())
      _thread.join();
  }
};

int main(int /*argc*/, char * /*args*/[]) {
  Display display(window_width, window_height);

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
  const auto pixels_per_worker =
      std::div(static_cast<int>(camera.pixels.size()), thread_count);

  std::vector<std::unique_ptr<Worker>> workers;
  workers.reserve(thread_count);

  for (unsigned i = 0; i < thread_count; i++) {
    const unsigned start = pixels_per_worker.quot * i;
    const unsigned count = pixels_per_worker.quot +
                           (i == thread_count - 1 ? pixels_per_worker.rem : 0);
    PixelChunk chunk = {.begin_index = static_cast<int>(start),
                        .end_index = static_cast<int>(start + count)};
    auto worker = std::make_unique<Worker>(std::ref(camera.pixels), chunk,
                                           std::cref(scene), std::cref(camera));
    worker->start_rendering();
    workers.push_back(std::move(worker));
  }

  while (display.is_running()) {
    bool is_rendering = false;
    for (auto &worker : workers)
      is_rendering = is_rendering || worker->is_rendering();
    display.draw_pixels(camera.pixels);
    display.update();
    if (!is_rendering)
      break;
  }

  for (auto &worker : workers)
    worker->stop_rendering();
  display.complete_measure("Rendered in ");

  while (display.is_running())
    display.wait_for_event();

  return 0;
}
