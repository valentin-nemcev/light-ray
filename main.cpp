#include <Eigen/Dense>
#include <SDL2/SDL.h>
#include <boost/format.hpp>
#include <iostream>
#include <stdexcept>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

using boost::format;
using Eigen::Vector2d;
using Eigen::Vector3d;

class Display {
    SDL_Window *window = nullptr;
    SDL_Surface *screenSurface = nullptr;

    void sdlError(std::string message) {
        throw std::runtime_error(
            boost::str(format("%s: %s\n") % message % SDL_GetError()));
    }

  public:
    Display() {
        if (SDL_Init(SDL_INIT_VIDEO) > 0) {
            sdlError("Could not initialize SDL2");
        }

        window = SDL_CreateWindow("Light Ray",
                                  SDL_WINDOWPOS_UNDEFINED,
                                  SDL_WINDOWPOS_UNDEFINED,
                                  SCREEN_WIDTH,
                                  SCREEN_HEIGHT,
                                  SDL_WINDOW_SHOWN);
        if (window == nullptr) {
            sdlError("Could not create window");
        }

        screenSurface = SDL_GetWindowSurface(window);
    }

    void setPixel(int x, int y, int r, int g, int b) {
        const SDL_Rect rect = {.x = x, .y = y, .w = 1, .h = 1};
        SDL_FillRect(
            screenSurface, &rect, SDL_MapRGB(screenSurface->format, r, g, b));
    }

    void update() { SDL_UpdateWindowSurface(window); }

    void waitForQuit() {
        for (bool running = true; running;) {
            SDL_Event event;
            while (SDL_PollEvent(&event) != 0) {
                if (event.type == SDL_QUIT) {
                    running = false;
                }
            }
        }

        SDL_DestroyWindow(window);
        SDL_Quit();
    }
};

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

struct Sphere {
    const Vector3d pos;
    const double radius;

    RayBounce intersect(const Ray &ray) const {
        const double discriminant =
            pow(ray.direction.dot(ray.origin - pos), 2) -
            (ray.origin - pos).squaredNorm() + pow(radius, 2);
        if (discriminant < 0) {
            return noBounce;
        }
        return {
            .distance =
                -ray.direction.dot(ray.origin - pos) - sqrt(discriminant),
        };
    }
};

struct CameraPixel {
    const Pixel pixel;
    const Ray ray;
    int value;

    void render(std::vector<Sphere> scene) {
        RayBounce closestBounce = noBounce;
        for (auto &shape : scene) {
            RayBounce bounce = shape.intersect(ray);
            if (bounce.bounced() &&
                (!closestBounce.bounced() ||
                 bounce.distance < closestBounce.distance)) {
                closestBounce = bounce;
            }
        }
        if (closestBounce.bounced()) {
            double hit = exp(-closestBounce.distance * 0.5);
            value = static_cast<int>(255.0 * hit);
        } else {
            value = 0;
        }
    }
};

class Camera {
  private:
    CameraPixel indexToPixel(const unsigned int index) const {
        const Pixel pixel(index % imageSize.x(), index / imageSize.x());
        const double aspectRatio =
            static_cast<double>(imageSize.x()) / imageSize.y();
        const Vector2d pixelPos(
            (pixel.x() + 0.5) / imageSize.x() - 0.5,
            (pixel.y() + 0.5) / imageSize.y() / aspectRatio - 0.5);

        Vector3d direction = (target - pos).normalized();
        const Vector3d cameraRight = direction.cross(directionUp);
        const Vector3d cameraUp = cameraRight.cross(direction);
        Vector3d rayDirection =
            (direction + cameraRight * pixelPos.x() - cameraUp * pixelPos.y())
                .normalized();
        return {.pixel = pixel,
                .ray = {.origin = pos, .direction = rayDirection}};
    }

  public:
    const Vector3d pos;
    const Vector3d target;
    const Pixel imageSize;

    std::vector<CameraPixel> render() const {
        std::vector<CameraPixel> pixels;
        const Pixel::Scalar pixelCount = imageSize.x() * imageSize.y();
        pixels.reserve(pixelCount);
        for (int i = 0; i < pixelCount; i++) {
            pixels.push_back(indexToPixel(i));
        }
        return pixels;
    }
};

int main(int /*argc*/, char * /*args*/ []) {
    Display display;

    const Camera camera = {
        .pos = Vector3d(3, 3, 1.5),
        .target = Vector3d(0, 0, 0),
        .imageSize = Pixel(SCREEN_WIDTH, SCREEN_HEIGHT),
    };

    const std::vector<Sphere> scene{
        {.pos = Vector3d(0.3, 0.3, 0.3), .radius = 0.3},
        {.pos = Vector3d(0, -0.5, 0.5), .radius = 0.5},
        {.pos = Vector3d(-0.6, 0, 0.6), .radius = 0.6},
    };

    auto pixels = camera.render();

    for (auto &pixel : pixels) {
        pixel.render(scene);
        display.setPixel(pixel.pixel.x(),
                         pixel.pixel.y(),
                         pixel.value,
                         pixel.value,
                         pixel.value);
    }

    display.update();
    display.waitForQuit();

    return 0;
}
