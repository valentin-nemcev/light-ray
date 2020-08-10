#pragma once
#include <boost/format/free_funcs.hpp>
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

class Counter {
  using f_seconds = std::chrono::duration<float, std::chrono::seconds::period>;
  int _count = 0;
  std::chrono::time_point<std::chrono::system_clock> _start_time;

  std::string _per_second_str = "-";

  int _reset_and_get_count() {
    _start_time = std::chrono::system_clock::now();
    int count = _count;
    _count = 0;
    return count;
  };

public:
  Counter() { _reset_and_get_count(); };

  void increment() { _count++; };

  std::string &per_second() {
    const auto seconds =
        std::chrono::duration<double, std::chrono::seconds::period>(
            std::chrono::system_clock::now() - _start_time);

    if (seconds.count() > 1.0) {
      _per_second_str = boost::str(boost::format("%4.1f") %
                                   (_reset_and_get_count() / seconds.count()));
    }

    return _per_second_str;
  }
};
