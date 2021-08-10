#pragma once

#include "renderer.hpp"
#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

class Histogram {
  std::vector<int> _buckets;
  std::vector<float> _normalized_buckets;
  int _max_value;
  int _total_count = 0;

  int _value_to_bucket(int value) {
    return std::clamp<int>(value / (1.0 * _max_value / _buckets.size()), 0,
                           _max_value);
  }

public:
  Histogram(int size = 0, int max_value = 0)
      : _buckets(size), _normalized_buckets(size), _max_value(max_value) {}

  void count_value(int value) {
    _buckets[_value_to_bucket(value)]++;
    _total_count++;
  }

  std::span<float> normalized_buckets() {
    std::transform(_buckets.begin(), _buckets.end(),
                   _normalized_buckets.begin(), [&](int value) -> float {
                     return std::pow(static_cast<float>(value) /
                                         static_cast<float>(_total_count),
                                     1.0 / 4);
                   });
    return std::span(_normalized_buckets);
  }
};

class Statistics {

  unsigned _count = 0;
  unsigned long _iterations = 0;
  double _avg_value = 0;
  double _avg_variance = 0;
  double _avg_std_error = 0;
  Histogram _value_histogram;

public:
  Statistics()
      : _value_histogram(16, std::numeric_limits<Uint8>::max() -
                                 std::numeric_limits<Uint8>::min() + 1) {}

  Histogram &value_histogram() { return _value_histogram; }

  void count_pixel(PixelDisplayValue &pixel) {
    _count++;
    _iterations += pixel.iterations;

    double pixel_weight = double(pixel.iterations) / _iterations;
    _avg_value += pixel_weight * (pixel.value - _avg_value);

    _avg_variance += pixel_weight * (pixel.variance - _avg_variance);

    _avg_std_error += pixel_weight * (pixel.std_error - _avg_std_error);

    _value_histogram.count_value(pixel.red);
  };

  [[nodiscard]] unsigned iterations_per_pixel() const {
    return _iterations / _count;
  }

  [[nodiscard]] double avg_value() const { return _avg_value; }

  [[nodiscard]] double std_dev() const { return std::sqrt(_avg_variance); }
  [[nodiscard]] double avg_std_error() const { return _avg_std_error; }
};
