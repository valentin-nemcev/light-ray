#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <boost/chrono.hpp>
#include <boost/format.hpp>
#include <csignal>
#include <iostream>
#include <ostream>
#include <random>
#include <thread>
#include <utility>

#include "display.hpp"
#include "renderer.hpp"
#include "stopwatch.hpp"

// default constructors reference
// https://en.cppreference.com/w/cpp/language/rule_of_three

constexpr unsigned window_width = 640;
constexpr unsigned window_height = 480;

;

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

  PixelChunk &take_next_chunk(const Worker *worker_ptr) {
    const std::lock_guard<std::mutex> lock(_queue_mutex);
    for (std::size_t i = 0; i < _queue.size(); i++) {
      PixelChunk &chunk = _queue[_next_chunk_index];
      _next_chunk_index = (_next_chunk_index + 1) % _queue.size();
      if (chunk.worker != nullptr)
        continue;
      chunk.worker = worker_ptr;
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

  boost::chrono::time_point<boost::chrono::thread_clock> _cpu_start_time;
  std::atomic<boost::chrono::thread_clock::duration> _cpu_duration{};

  void _render() {
    for (;;) {
      PixelChunk &pixel_chunk = _queue.take_next_chunk(this);
      bool did_work = false;
      for (unsigned pixel_index = pixel_chunk.begin_index;
           pixel_index < pixel_chunk.end_index; pixel_index++) {

        auto &pixel = _pixels[pixel_index];
        if (Renderer::render_pixel(_scene, _camera, pixel, pixel_index))
          did_work = true;
        if (!_is_rendering)
          break;
      }
      _update_cpu_duration();
      _queue.return_chunk(pixel_chunk);
      if (!_is_rendering)
        break;
      if (!did_work)
        break;
    }
    _is_rendering = false;
  }

  void _update_cpu_duration() {
    _cpu_duration = boost::chrono::thread_clock::now() - _cpu_start_time;
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
    _cpu_start_time = boost::chrono::thread_clock::now();
  }

  void stop_rendering() {
    _is_rendering = false;
    if (_thread.joinable())
      _thread.join();
  }

  boost::chrono::thread_clock::duration cpu_duration() const {
    return _cpu_duration;
  }
};

Scene create_scene() {
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
      std::make_unique<SkySurface>(Vector3d(-4, 6, 4))));
  return scene;
}

// TODO:
// https://stackoverflow.com/questions/32257840/properly-terminating-program-using-exceptions
// NOLINTNEXTLINE(bugprone-exception-escape)
int main(int /*argc*/, char * /*args*/[]) {
  Stopwatch stopwatch("Allocating display");
  Display display(window_width, window_height, 1);

  auto screen_dimensions = display.screen_dimensions();

  std::cout << boost::format("Image dimensions: %dx%d\n") %
                   screen_dimensions.width % screen_dimensions.height;

  // Camera camera(Vector3d(4, 2, 1.5), Vector3d(0, 0, 0.5),
  //               Vector2i(screen_dimensions.width, screen_dimensions.height));

  stopwatch("Allocating camera");
  Camera camera(Vector3d(2, 3, 4), Vector3d(0, 0, 0.5),
                Vector2i(screen_dimensions.width, screen_dimensions.height));

  display.set_pixels(&camera.pixels);

  stopwatch("Allocating scene");

  Scene scene = create_scene();

  stopwatch("Allocating and starting workers");

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

  stopwatch("Rendering");
  bool is_rendering = true;
  while (display.is_running()) {
    boost::chrono::thread_clock::duration cpu_duration =
        boost::chrono::nanoseconds::zero();
    bool rendering_done = true;
    for (auto &worker : workers) {
      rendering_done = rendering_done && !worker->is_rendering();
      cpu_duration += worker->cpu_duration();
    }

    if (is_rendering && rendering_done) {
      is_rendering = false;
      stopwatch("Idle");
    }

    display.set_stats(
        boost::chrono::duration_cast<boost::chrono::seconds>(cpu_duration)
            .count());
    display.draw_pixels();
    display.update();
  }

  for (auto &worker : workers)
    worker->stop_rendering();
  stopwatch.stop();

  return 0;
}
