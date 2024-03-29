#include "accelerators/cloud.h"
#include "cloud/manager.h"
#include "core/camera.h"
#include "core/geometry.h"
#include "core/sampler.h"
#include "core/stats.h"
#include "integrators/cloud.h"
#include "lights/diffuse.h"
#include "lights/pinfinite.h"
#include "messages/serdes.h"
#include "messages/utils.h"
#include "pbrt/main.h"
#include "pbrt/raystate.h"
#include "util/defer.h"

using namespace std;

namespace pbrt {

static auto &_manager = global::manager;

STAT_COUNTER("Integrator/Camera rays generated", nCameraRays);
STAT_COUNTER("Integrator/Total rays traced", totalRays);
STAT_COUNTER("Intersections/Regular ray intersection tests",
             nIntersectionTests);
STAT_COUNTER("Integrator/Calls to Process", nProcessRayCalls);
STAT_COUNTER("Integrator/Total Process time", totalProcessTime);

void AccumulatedStats::Merge(const AccumulatedStats &other) {
    for (const auto &item : other.counters) {
        counters[item.first] += item.second;
    }

    for (const auto &item : other.memoryCounters) {
        memoryCounters[item.first] += item.second;
    }

    for (const auto &item : other.intDistributionCounts) {
        intDistributionCounts[item.first] += item.second;
        intDistributionSums[item.first] +=
            other.intDistributionSums.at(item.first);

        if (intDistributionMins.count(item.first) == 0) {
            intDistributionMins[item.first] =
                other.intDistributionMins.at(item.first);
        } else {
            intDistributionMins[item.first] =
                min(intDistributionMins[item.first],
                    other.intDistributionMins.at(item.first));
        }

        if (intDistributionMaxs.count(item.first) == 0) {
            intDistributionMaxs[item.first] =
                other.intDistributionMaxs.at(item.first);
        } else {
            intDistributionMaxs[item.first] =
                max(intDistributionMaxs[item.first],
                    other.intDistributionMaxs.at(item.first));
        }
    }

    for (const auto &item : other.floatDistributionCounts) {
        floatDistributionCounts[item.first] += item.second;
        floatDistributionSums[item.first] +=
            other.floatDistributionSums.at(item.first);

        if (!floatDistributionMins.count(item.first)) {
            floatDistributionMins[item.first] =
                other.floatDistributionMins.at(item.first);
        } else {
            floatDistributionMins[item.first] =
                min(floatDistributionMins.at(item.first),
                    other.floatDistributionMins.at(item.first));
        }

        if (!floatDistributionMaxs.count(item.first)) {
            floatDistributionMaxs[item.first] =
                other.floatDistributionMaxs.at(item.first);
        } else {
            floatDistributionMaxs[item.first] =
                max(floatDistributionMaxs[item.first],
                    other.floatDistributionMaxs.at(item.first));
        }
    }

    for (const auto &item : other.percentages) {
        percentages[item.first].first += item.second.first;
        percentages[item.first].second += item.second.second;
    }

    for (const auto &item : other.ratios) {
        ratios[item.first].first += item.second.first;
        ratios[item.first].second += item.second.second;
    }
}

string GetObjectName(const ObjectType type, const uint32_t id) {
    return SceneManager::getFileName(type, id);
}

SceneBase::SceneBase(const std::string &path, const int samplesPerPixel) {
    using namespace pbrt::global;

    PbrtOptions.nThreads = 1;
    manager.init(path);

    auto reader = manager.GetReader(ObjectType::Camera);
    protobuf::Camera proto_camera;
    reader->read(&proto_camera);
    camera = camera::from_protobuf(proto_camera, transformCache);

    reader = manager.GetReader(ObjectType::Sampler);
    protobuf::Sampler proto_sampler;
    reader->read(&proto_sampler);
    sampler = sampler::from_protobuf(proto_sampler, samplesPerPixel);

    vector<shared_ptr<Light>> lights;

    // loading area lights
    MediumInterface mediumInterface{};
    reader = manager.GetReader(ObjectType::AreaLights);

    while (!reader->eof()) {
        protobuf::AreaLight proto_light;
        reader->read(&proto_light);

        // let's create the triangle mesh for the light
        unique_ptr<char[]> meshStorage{
            make_unique<char[]>(proto_light.mesh_data().size())};

        memcpy(meshStorage.get(), &proto_light.mesh_data()[0],
               proto_light.mesh_data().size());
        areaLightMeshes.emplace_back(
            make_shared<TriangleMesh>(move(meshStorage), 0));
        auto mesh = areaLightMeshes.back();

        auto lightParams = from_protobuf(proto_light.light().paramset());
        auto lightTrans = from_protobuf(proto_light.light().light_to_world());

        for (size_t i = 0; i < mesh->nTriangles; i++) {
            areaLightShapes.emplace_back(make_shared<Triangle>(
                &identityTransform, &identityTransform, false, mesh, i));

            lights.emplace_back(
                CreateDiffuseAreaLight(lightTrans, mediumInterface.outside,
                                       lightParams, areaLightShapes.back()));
        }
    }

    // loading normal lights
    reader = manager.GetReader(ObjectType::Lights);
    while (!reader->eof()) {
        protobuf::Light proto_light;
        reader->read(&proto_light);
        lights.push_back(move(light::from_protobuf(proto_light)));
    }

    for (uint32_t i = 0; i < lights.size(); i++) {
        lights[i]->SetID(i + 1);
    }

    // loading infinite lights with texture maps
    reader = manager.GetReader(ObjectType::InfiniteLights);
    while (!reader->eof()) {
        protobuf::InfiniteLight proto_light;
        reader->read(&proto_light);

        auto &envmap_proto = proto_light.environment_map();

        vector<uint32_t> treeletMapping{
            envmap_proto.partition_treelets().begin(),
            envmap_proto.partition_treelets().end()};

        lights.emplace_back(std::make_shared<CloudInfiniteAreaLight>(
            from_protobuf(proto_light.light().light_to_world()),
            from_protobuf(proto_light.power()), 1,
            reinterpret_cast<const RGBSpectrum *>(
                envmap_proto.downsampled_image().data()),
            from_protobuf(envmap_proto.downsampled_image_resolution()),
            from_protobuf(envmap_proto.resolution()), treeletMapping));

        lights.back()->SetID(lights.size());
    }

    reader = manager.GetReader(ObjectType::Scene);
    protobuf::Scene proto_scene;
    reader->read(&proto_scene);
    fakeScene = make_unique<Scene>(from_protobuf(proto_scene, move(lights)));

    const auto treeletCount = manager.treeletCount();
    treeletDependencies.resize(treeletCount);

    for (TreeletId i = 0; i < treeletCount; i++) {
        treeletDependencies[i] = manager.getTreeletDependencies(i);
    }

    this->samplesPerPixel = sampler->samplesPerPixel;
    sampleBounds = camera->film->GetSampleBounds();
    sampleExtent = sampleBounds.Diagonal();
    totalPaths = sampleBounds.Area() * sampler->samplesPerPixel;
}

SceneBase LoadSceneBase(const std::string &path, const int samplesPerPixel) {
    return {path, samplesPerPixel};
}

shared_ptr<CloudBVH> LoadTreelet(const string &path, const TreeletId treeletId,
                                 const char *buffer, const size_t length) {
    using namespace pbrt::global;
    manager.init(path);
    shared_ptr<CloudBVH> treelet = make_shared<CloudBVH>(treeletId, false);
    treelet->LoadTreelet(treeletId, buffer, length);
    return treelet;
}

set<ObjectKey> &SceneBase::TreeletDependencies(const TreeletId treeletId) {
    return treeletDependencies.at(treeletId);
}

void SceneBase::ProcessRay(RayStatePtr &&rayStatePtr, const CloudBVH &treelet,
                           MemoryArena &arena, ProcessRayOutput &output) {
    auto _ = defer([start_time = chrono::steady_clock::now()] {
        const auto end_time = chrono::steady_clock::now();
        totalProcessTime +=
            chrono::duration_cast<chrono::nanoseconds>(end_time - start_time)
                .count();
    });

    nProcessRayCalls++;
    auto &r = *rayStatePtr;

    output.pathId = r.PathID();
    output.pathFinished = false;
    output.rays[0] = nullptr;
    output.rays[1] = nullptr;
    output.rays[2] = nullptr;
    output.sample = nullptr;

    RayStatePtr tracedRay;

    if (!r.toVisitEmpty()) {
        tracedRay = CloudIntegrator::Trace(move(rayStatePtr), treelet);
    } else if (r.HasHit()) {
        RayStatePtr bounceRay, shadowRay, lightRay;

        tie(bounceRay, shadowRay, lightRay) =
            CloudIntegrator::Shade(move(rayStatePtr), treelet, *fakeScene,
                                   sampleExtent, sampler, maxPathDepth, arena);

        if (!bounceRay and !shadowRay) {
            output.pathFinished = true;
            return;
        }

        if (bounceRay) output.rays[0] = move(bounceRay);
        if (shadowRay) output.rays[1] = move(shadowRay);
        if (lightRay) output.rays[2] = move(lightRay);
        return;
    } else if (r.needsImageSampling) {
        auto &p = _manager.getInMemoryImagePartition(r.imageSampleInfo.imageId);
        const auto Li = p.Lookup(r.imageSampleInfo.uv);
        r.Ld *= Li;

        if ((r.IsShadowRay() and r.remainingBounces == 0) ||
            (not r.IsShadowRay() and not r.IsLightRay() and
             r.remainingBounces == maxPathDepth - 1)) {
            output.pathFinished = true;
        }

        output.sample = move(rayStatePtr);
        return;
    } else {
        throw runtime_error("ProcessRay: invalid ray");
    }

    if (!tracedRay) {
        throw runtime_error("traced ray cannot be null");
    }

    auto &ray = *tracedRay;
    const bool hit = ray.HasHit();
    const bool emptyVisit = ray.toVisitEmpty();

    if (emptyVisit && !hit && ray.needsImageSampling) {
        // we should send the ray for image sampling
        output.rays[0] = move(tracedRay);
        return;
    }

    if (ray.IsShadowRay()) {
        if (hit or emptyVisit) {
            /* was this the last shadow ray? */
            if (ray.remainingBounces == 0) {
                output.pathFinished = true;
            }

            ray.Ld = hit ? 0.f : ray.Ld;
            output.sample = move(tracedRay);
        } else {
            output.rays[0] = move(tracedRay);
        }
    } else if (ray.IsLightRay()) {
        if (emptyVisit) {
            Spectrum Li{0.f};
            const auto sLight = ray.lightRayInfo.sampledLightId;

            if (hit) {
                const auto aLight = ray.hitInfo.arealight;

                if (aLight == sLight) {
                    Li = dynamic_cast<AreaLight *>(
                             fakeScene->lights[aLight - 1].get())
                             ->L(ray.hitInfo.isect,
                                 -ray.lightRayInfo.sampledDirection);
                }
            } else {
                auto &l = fakeScene->lights[sLight - 1];
                if (l->GetType() != LightType::PartitionedInfinite) {
                    Li = l->Le(ray.ray);
                } else {
                    bool isBlack = false;
                    auto pLight =
                        dynamic_cast<CloudInfiniteAreaLight *>(l.get());
                    auto uv = pLight->Le_SampledPoint(ray.ray);
                    auto img = pLight->GetPointImageInfo(uv, isBlack);
                    if (!isBlack) {
                        ray.needsImageSampling = true;
                        ray.imageSampleInfo.uv = uv;
                        ray.imageSampleInfo.treelet = img.first;
                        ray.imageSampleInfo.imageId = img.second;
                        output.rays[0] = move(tracedRay);
                    }
                }
            }

            if (!Li.IsBlack()) {
                ray.Ld *= Li;
                output.sample = move(tracedRay);
            }
        } else {
            output.rays[0] = move(tracedRay);
        }
    } else if (!emptyVisit or hit) {
        output.rays[0] = move(tracedRay);
    } else if (emptyVisit) {
        ray.Ld = 0.f;

        bool sampleDone = true;
        if (ray.remainingBounces == maxPathDepth - 1) {
            for (const auto &light : fakeScene->infiniteLights) {
                if (light->GetType() == LightType::PartitionedInfinite) {
                    ray.Ld = 1.f;

                    bool isBlack = false;
                    auto pLight =
                        dynamic_cast<CloudInfiniteAreaLight *>(light.get());
                    const auto uv = pLight->Le_SampledPoint(ray.ray);
                    const auto img = pLight->GetPointImageInfo(uv, isBlack);

                    if (!isBlack) {
                        ray.needsImageSampling = true;
                        ray.imageSampleInfo.uv = uv;
                        ray.imageSampleInfo.treelet = img.first;
                        ray.imageSampleInfo.imageId = img.second;
                        output.rays[0] = move(tracedRay);
                        sampleDone = false;
                    } else {
                        ray.Ld = 0.f;
                        sampleDone = true;
                    }

                    break;
                } else {
                    ray.Ld += light->Le(ray.ray);
                }
            }
        }

        if (sampleDone) {
            output.pathFinished = true;
            output.sample = move(tracedRay);
        }
    }
}

RayStatePtr SceneBase::GenerateCameraRay(const Point2i &pixel,
                                         const uint32_t sample) {
    if (!InsideExclusive(pixel, sampleBounds)) {
        return nullptr;
    }

    const Float rayScale = 1 / sqrt((Float)samplesPerPixel);

    sampler->StartPixel(pixel);
    sampler->SetSampleNumber(sample);

    CameraSample cameraSample = sampler->GetCameraSample(pixel);

    RayStatePtr statePtr = RayState::Create();
    RayState &state = *statePtr;

    state.sample.id =
        (pixel.x + pixel.y * sampleExtent.x) * samplesPerPixel + sample;
    state.sample.dim = sampler->GetCurrentDimension();
    state.sample.pFilm = cameraSample.pFilm;
    state.sample.weight =
        camera->GenerateRayDifferential(cameraSample, &state.ray);
    state.ray.ScaleDifferentials(rayScale);
    state.remainingBounces = maxPathDepth - 1;
    state.StartTrace();

    ++nCameraRays;
    ++nIntersectionTests;
    ++totalRays;

    return statePtr;
}

void SceneBase::AccumulateImage(const vector<Sample> &rays) {
    const Bounds2i sampleBounds = camera->film->GetSampleBounds();
    unique_ptr<FilmTile> filmTile = camera->film->GetFilmTile(sampleBounds);

    for (const auto &ray : rays) {
        filmTile->AddSample(ray.pFilm, ray.L, ray.weight, true);
    }

    camera->film->MergeFilmTile(move(filmTile));
}

void SceneBase::WriteImage(const string &filename) {
    if (not filename.empty()) {
        camera->film->SetFilename(filename);
    }

    camera->film->WriteImage();
}

}  // namespace pbrt
