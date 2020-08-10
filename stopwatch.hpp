#pragma once
#include <chrono>
#include <iostream>
#include <ostream>
#include <string>

class Stopwatch {
  std::string _message;
  std::chrono::time_point<std::chrono::system_clock> _start_time;
  bool _is_running = false;

public:
  Stopwatch() = delete;
  Stopwatch(const Stopwatch &) = delete;
  Stopwatch(Stopwatch &&) = delete;
  Stopwatch &operator=(const Stopwatch &) = delete;
  Stopwatch &operator=(Stopwatch &&) = delete;

  Stopwatch(const std::string &message) { start(message); };
  ~Stopwatch() { stop(); };

  void operator()(const std::string &message) { start(message); };

  void start(std::string message) {
    stop();
    _message = std::move(message);
    std::cout << _message << "... " << std::endl;
    _is_running = true;
    _start_time = std::chrono::system_clock::now();
  };

  void stop() {
    if (!_is_running)
      return;
    const auto duration = std::chrono::system_clock::now() - _start_time;
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::cout << _message << " done in " << ms << "ms" << std::endl;
    _is_running = false;
  };
};
