#include <FastBVH.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "Log.h"
#include "SimpleScheduler.h"
#include "Stopwatch.h"

namespace {

using std::vector;

using namespace FastBVH;

// Return a random number in [0,1]
float rand01() { return rand() * (1.f / RAND_MAX); }

// Return a random vector with each component in the range [-1,1]
Vector3<float> randVector3() { return Vector3<float>{rand01(), rand01(), rand01()} * 2.0f - Vector3<float>{1, 1, 1}; }

//! For the purposes of demonstrating the BVH, a simple sphere
template <typename Float>
struct Sphere final {
  Vector3<Float> center;  // Center of the sphere

  Float r, r2;            // Radius, Radius^2

  constexpr Sphere(const Vector3<Float>& center, Float radius) noexcept
      : center(center), r(radius), r2(radius * radius) {}
};

//! \brief Used for calculating the bounding boxes
//! associated with spheres.
//! \tparam Float The floating point type of the sphere and bounding box vectors.
template <typename Float>
class SphereBoxConverter final {
 public:
  //! Converts a sphere to a bounding box.
  //! \param sphere The sphere to convert to a bounding box.
  //! \return A bounding box that encapsulates the sphere.
  BBox<Float> operator()(const Sphere<Float>& sphere) const noexcept {
    const auto& r = sphere.r;

    auto box_delta = Vector3<Float>{r, r, r};

    return BBox<Float>(sphere.center - box_delta, sphere.center + box_delta);
  }
};

//! \brief Used for calculating the squared distance between ray orgins and spheres.
//! \tparam Float The floating point type of the spheres and rays.
template <typename Float>
class SphereIntersector final {
public:
    //! Compute squared distance between ray origin and the sphere primitive
    //! \param sphere The sphere to check intersection for.
    //! \param ray The ray whose origin is being inquired for distance to the sphere
    //! \return An instance of @ref FastBVH::Intersection that contains the distance
    //! data.
    Intersection<Float, Sphere<Float>> operator()(const Sphere<Float>& sphere, const Ray<Float>& ray) const noexcept {
        auto oc = length(sphere.center - ray.o);
        auto os = fabs(oc - sphere.r);

        return Intersection<Float, Sphere<Float>>{os*os, &sphere};
    }
};

}  // namespace

int main() {
  // Create a million spheres packed in the space of a cube
  const unsigned int N = 1000000;

  vector<Sphere<float>> objects;

  printf("Constructing %d spheres...\n", N);
  for (size_t i = 0; i < N; ++i) {
    objects.emplace_back(Sphere<float>(randVector3(), .005f));
  }

  SphereBoxConverter<float> box_converter;

  Stopwatch stopwatch;

  FastBVH::BuildStrategy<float, 1> build_strategy;

  auto bvh = build_strategy(objects, box_converter);

  // Output tree build time and statistics
  double constructionTime = stopwatch.read();

  LOG_STAT("Built BVH (%u nodes, with %u leafs) in %.02f ms", (unsigned int)bvh.getNodes().size(),
           (unsigned int)bvh.countLeafs(), 1000.0 * constructionTime);

  SphereIntersector<float> intersector;
  Traverser<float, Sphere<float>, decltype(intersector)> traverser(bvh, intersector);

    auto trace_kernel = [traverser](const Ray<float>& ray) {
        auto isect = traverser.traverse(ray);
        auto dist = sqrt(isect.t);
        // Just for fun, we'll make the color based on the reciprocal of distance
        return Vector3<float>{0.18f/dist, 0.18f/dist, 0.18f/dist};
    };

  constexpr std::size_t width = 512;
  constexpr std::size_t height = 512;

  printf("Rendering image (%ux%u)...\n", (unsigned int)width, (unsigned int)height);

  stopwatch.reset();
  SimpleScheduler<float> scheduler(width, height);
  scheduler.schedule(trace_kernel);
  double render_time = stopwatch.read();

  printf("Rendered %lu primary rays in %.2f seconds (%.2f MRays/sec)\n", width * height, render_time,
         width * height / render_time / 1000000.0);
  scheduler.saveResults("render.ppm");
}
