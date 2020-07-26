#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <random>
#include <variant>

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

