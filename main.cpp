#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <boost/format.hpp>
#include <csignal>
#include <iostream>
#include <ostream>
#include <random>
#include <thread>

#include "display.hpp"
#include "renderer.hpp"

// default constructors reference
// https://en.cppreference.com/w/cpp/language/rule_of_three

constexpr unsigned window_width = 640;
constexpr unsigned window_height = 480;

class Worker;

struct PixelChunk {
  unsigned begin_index{};
  unsigned end_index{};
  const Worker *worker = nullptr;
};

class ChunkQueue {
  std::vector<PixelChunk> _queue;
  std::size_t _next_chunk_index = 0;
  std::mutex _queue_mutex;

  [[nodiscard]] static std::vector<PixelChunk>
  _allocate_queue(const unsigned pixel_count, const unsigned chunk_count) {
    std::vector<PixelChunk> queue;
    const auto pixels_per_chunk = (pixel_count / chunk_count);
    queue.reserve(chunk_count);
    for (unsigned chunk = 0; chunk < chunk_count; chunk++) {
      const unsigned begin_index = pixels_per_chunk * chunk;
      const unsigned end_index = begin_index + pixels_per_chunk;
      queue.push_back({.begin_index = begin_index, .end_index = end_index});
    }
    std::shuffle(queue.begin(), queue.end(), random_engine);
    return queue;
  }

public:
  ChunkQueue() = delete;
  ChunkQueue(const ChunkQueue &) = delete;
  ChunkQueue(ChunkQueue &&) = delete;
  ChunkQueue &operator=(const ChunkQueue &) = delete;
  ChunkQueue &operator=(ChunkQueue &&) = delete;
  ~ChunkQueue() = default;

  ChunkQueue(const unsigned pixel_count, const unsigned chunk_count)
      : _queue(_allocate_queue(pixel_count, chunk_count)){};

  PixelChunk &take_next_chunk(const Worker *worker) {
    const std::lock_guard<std::mutex> lock(_queue_mutex);
    for (std::size_t i = 0; i < _queue.size(); i++) {
      PixelChunk &chunk = _queue[_next_chunk_index];
      _next_chunk_index = (_next_chunk_index + 1) % _queue.size();
      if (chunk.worker != nullptr)
        continue;
      chunk.worker = worker;
      return chunk;
    }
    throw std::runtime_error("Render queue drained");
  };

  void return_chunk(PixelChunk &chunk) {
    const std::lock_guard<std::mutex> lock(_queue_mutex);
    chunk.worker = nullptr;
  }
};

class Worker {
  Pixels &_pixels;
  ChunkQueue &_queue;
  SceneRef _scene;
  const Camera &_camera;

  std::thread _thread;
  std::atomic<bool> _is_rendering = false;

  void _render() {
    for (;;) {
      PixelChunk &pixel_chunk = _queue.take_next_chunk(this);
      unsigned iterations_done = 0;
      for (unsigned pixel_index = pixel_chunk.begin_index;
           pixel_index < pixel_chunk.end_index; pixel_index++) {

        auto &pixel = _pixels[pixel_index];
        iterations_done +=
            Renderer::render_pixel(_scene, _camera, pixel, pixel_index);
        if (!_is_rendering)
          break;
      }
      _queue.return_chunk(pixel_chunk);
      if (!_is_rendering)
        break;
      if (iterations_done == 0)
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

  Worker(Pixels &pixels, ChunkQueue &queue, SceneRef scene,
         const Camera &camera)
      : _pixels(pixels), _queue(queue), _scene(scene), _camera(camera) {}

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
  Display display(window_width, window_height, 1);

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

  std::vector<std::unique_ptr<Worker>> workers;
  workers.reserve(thread_count);

  ChunkQueue queue(camera.pixels.size(), camera.image_size.y());

  for (unsigned i = 0; i < thread_count; i++) {
    auto worker =
        std::make_unique<Worker>(std::ref(camera.pixels), std::ref(queue),
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
