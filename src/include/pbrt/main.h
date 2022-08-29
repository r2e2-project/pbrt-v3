#ifndef PBRT_INCLUDE_MAIN_H
#define PBRT_INCLUDE_MAIN_H

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "common.h"
#include "geometry.h"
#include "light.h"
#include "pbrt.h"
#include "raystate.h"
#include "scene.h"
#include "transform.h"

namespace pbrt {

class TriangleMesh;
class GlobalSampler;
class CloudBVH;
class Sample;
class RayState;
using RayStatePtr = std::unique_ptr<RayState>;

struct ProcessRayOutput {
    uint64_t pathId{0};
    std::array<RayStatePtr, 3> rays{nullptr, nullptr, nullptr};
    RayStatePtr sample{};
    bool pathFinished{false};
};

class SceneBase {
  public:
    SceneBase() {}
    SceneBase(const std::string &path, const int samplesPerPixel);

    RayStatePtr GenerateCameraRay(const Point2i &pixel, const uint32_t sample);
    void AccumulateImage(const std::vector<Sample> &rays);
    void WriteImage(const std::string &filename = {});
    void ProcessRay(RayStatePtr &&ray, const CloudBVH &treelet,
                    MemoryArena &arena, ProcessRayOutput &output);

    std::set<ObjectKey> &TreeletDependencies(const TreeletId treeletId);
    size_t TreeletCount() const { return treeletDependencies.size(); }
    size_t MaxPathDepth() const { return maxPathDepth; }
    int SamplesPerPixel() const { return samplesPerPixel; }
    const Bounds2i &SampleBounds() const { return sampleBounds; }
    std::shared_ptr<pbrt::Camera> &Camera() { return camera; }

    void SetPathDepth(const size_t d) { maxPathDepth = d; }

    SceneBase(SceneBase &&) = default;
    SceneBase &operator=(SceneBase &&) = default;
    SceneBase &operator=(const SceneBase &) = delete;

  private:
    std::vector<std::set<ObjectKey>> treeletDependencies{};
    Transform identityTransform;

    std::shared_ptr<pbrt::Camera> camera{};
    std::shared_ptr<GlobalSampler> sampler{};
    std::vector<std::unique_ptr<Transform>> transformCache{};
    std::unique_ptr<Scene> fakeScene{};

    std::vector<std::shared_ptr<TriangleMesh>> areaLightMeshes{};
    std::vector<std::shared_ptr<Shape>> areaLightShapes{};

    int samplesPerPixel{};
    Bounds2i sampleBounds{};
    Vector2i sampleExtent{};
    size_t totalPaths{0};
    size_t maxPathDepth{5};
};

std::string GetObjectName(const ObjectType type, const uint32_t id);
SceneBase LoadSceneBase(const std::string &path, const int samplesPerPixel);

std::shared_ptr<CloudBVH> LoadTreelet(const std::string &path,
                                      const TreeletId treeletId,
                                      const char *buffer = nullptr,
                                      const size_t length = 0);

void DumpSceneObjects(const std::string &description,
                      const std::string outputPath);

struct AccumulatedStats {
    std::map<std::string, int64_t> counters{};
    std::map<std::string, int64_t> memoryCounters{};
    std::map<std::string, int64_t> intDistributionSums{};
    std::map<std::string, int64_t> intDistributionCounts{};
    std::map<std::string, int64_t> intDistributionMins{};
    std::map<std::string, int64_t> intDistributionMaxs{};
    std::map<std::string, double> floatDistributionSums{};
    std::map<std::string, int64_t> floatDistributionCounts{};
    std::map<std::string, double> floatDistributionMins{};
    std::map<std::string, double> floatDistributionMaxs{};
    std::map<std::string, std::pair<int64_t, int64_t>> percentages{};
    std::map<std::string, std::pair<int64_t, int64_t>> ratios{};

    void Merge(const AccumulatedStats &other);
};

namespace stats {

AccumulatedStats GetThreadStats();

}

}  // namespace pbrt

#endif /* PBRT_CLOUD_GRAPHICS_H */
