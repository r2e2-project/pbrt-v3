#include "cloud.h"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iterator>

#include "accelerators/cloud.h"
#include "cloud/manager.h"
#include "core/paramset.h"
#include "lights/pinfinite.h"
#include "materials/matte.h"

using namespace std;

namespace pbrt {

STAT_COUNTER("Integrator/Camera rays generated", nCameraRays);
STAT_COUNTER("Intersections/Regular ray intersection tests",
             nIntersectionTests);
STAT_COUNTER("Intersections/Shadow ray intersection tests", nShadowTests);
STAT_COUNTER("Integrator/Total rays traced", totalRays);
STAT_INT_DISTRIBUTION("Integrator/Unused bounces per path", nRemainingBounces);

STAT_COUNTER("Integrator/Calls to Shade", nShadeCalls);
STAT_COUNTER("Integrator/Calls to Trace", nTraceCalls);

RayStatePtr CloudIntegrator::Trace(RayStatePtr &&rayState,
                                   const CloudBVH &treelet) {
    nTraceCalls++;
    treelet.Trace(*rayState);

    if (!rayState->isShadowRay && rayState->toVisitEmpty() && !rayState->hit) {
        ReportValue(nRemainingBounces, rayState->remainingBounces);
    }

    return move(rayState);
}

tuple<RayStatePtr, RayStatePtr, RayStatePtr> CloudIntegrator::Shade(
    RayStatePtr &&rayStatePtr, const CloudBVH &treelet, const Scene &scene,
    const Vector2i &sampleExtent, shared_ptr<GlobalSampler> &sampler,
    int maxPathDepth, MemoryArena &arena) {
    nShadeCalls++;

    static thread_local unique_ptr<LightDistribution> lightDistribution =
        CreateLightSampleDistribution("spatial", scene);

    RayStatePtr bouncePtr = nullptr;
    RayStatePtr shadowRayPtr = nullptr;
    RayStatePtr lightRayPtr = nullptr;

    auto &rayState = *rayStatePtr;

    SurfaceInteraction &it = rayState.hitInfo.isect;

    if (rayState.hitInfo.material.id) {
        const auto material = treelet.GetMaterial(rayState.hitInfo.material.id);

        // the next two lines are basically:
        // it.ComputeScatteringFunctions(rayState.ray, arena, true);
        it.ComputeDifferentials(rayState.ray);
        if (material) {
            material->ComputeScatteringFunctions(
                &it, arena, pbrt::TransportMode::Radiance, true);
        }
    }

    if (!it.bsdf) {
        // Skipping intersection due to null bsdf
        bouncePtr = move(rayStatePtr);
        rayStatePtr = nullptr;
        auto &newRay = *bouncePtr;

        newRay.ray = it.SpawnRay(newRay.ray.d);
        newRay.StartTrace();

        return {move(bouncePtr), nullptr, nullptr};
    }

    /* setting the sampler */
    sampler->StartPixel(
        rayState.SamplePixel(sampleExtent, sampler->samplesPerPixel));
    sampler->SetSampleNumber(rayState.SampleNum(sampler->samplesPerPixel));
    sampler->SetDimension(rayState.sample.dim);

    const auto bsdfFlags = BxDFType(BSDF_ALL & ~BSDF_SPECULAR);

    const Distribution1D *distrib = lightDistribution->Lookup(it.p);

    if (it.bsdf->NumComponents(bsdfFlags) > 0 && scene.lights.size() > 0) {
        /* Let's pick a light at random */
        Float lightSelectPdf;
        const int lightNum =
            distrib->SampleDiscrete(sampler->Get1D(), &lightSelectPdf);

        const shared_ptr<Light> &light = scene.lights[lightNum];

        Point2f uLight = sampler->Get2D();
        Point2f uScattering = sampler->Get2D();  // For consistency with PBRT
        Float scatteringPdf = 0;
        Vector3f wi;
        Float lightPdf;
        VisibilityTester visibility;

        Spectrum Li{1.f};
        Optional<RayState::ImageSampleInfo> imageSampleInfo;

        if (light->GetType() == LightType::PartitionedInfinite) {
            bool isBlack = false;
            auto pLight = dynamic_cast<CloudInfiniteAreaLight *>(light.get());

            // to get this light's contribution, we need to sample the
            // envrionment map at this coordinates
            const auto p = pLight->Sample_Li_SampledPoint(
                it, uLight, &wi, &lightPdf, &visibility, isBlack);

            if (isBlack) {
                Li = 0.f;
            } else {
                Li = 1.f;
                auto sampleId = pLight->GetPointImageInfo(p, isBlack);
                if (isBlack) {
                    Li = 0.f;
                } else {
                    imageSampleInfo.initialize();
                    imageSampleInfo->uv = p;
                    imageSampleInfo->treelet = sampleId.first;
                    imageSampleInfo->imageId = sampleId.second;
                }
            }
        } else {
            Li = light->Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
        }

        ++totalRays;

        if (lightPdf > 0 && !Li.IsBlack()) {
            Spectrum f;
            f = it.bsdf->f(it.wo, wi, bsdfFlags) * AbsDot(wi, it.shading.n);
            scatteringPdf = it.bsdf->Pdf(it.wo, wi, bsdfFlags);

            if (!f.IsBlack()) {
                /* now we have to shoot the ray to the light source */
                shadowRayPtr = RayState::Create();
                auto &shadowRay = *shadowRayPtr;

                shadowRay.trackRay = rayState.trackRay;
                shadowRay.hop = 0;
                shadowRay.pathHop = rayState.pathHop;
                shadowRay.sample = rayState.sample;
                shadowRay.ray = visibility.P0().SpawnRayTo(visibility.P1());
                shadowRay.beta = rayState.beta;

                if (imageSampleInfo.initialized()) {
                    shadowRay.needsImageSampling = true;
                    shadowRay.imageSampleInfo = *imageSampleInfo;
                }

                if (IsDeltaLight(light->flags)) {
                    shadowRay.Ld = (f * Li / lightPdf) / lightSelectPdf;
                } else {
                    const Float weight =
                        PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    shadowRay.Ld =
                        (f * Li * weight / lightPdf) / lightSelectPdf;
                }

                shadowRay.remainingBounces = rayState.remainingBounces;
                shadowRay.isShadowRay = true;
                shadowRay.StartTrace();

                ++nShadowTests;
            }
        }

        if (!IsDeltaLight(light->flags)) {
            BxDFType sampledType;
            Spectrum f =
                it.bsdf->Sample_f(it.wo, &wi, uScattering, &scatteringPdf,
                                  bsdfFlags, &sampledType);
            f *= AbsDot(wi, it.shading.n);
            const bool sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;

            if (!f.IsBlack() && scatteringPdf) {
                if (!sampledSpecular) {
                    lightPdf = light->Pdf_Li(it, wi);

                    if (lightPdf > 0) {
                        const Float weight =
                            PowerHeuristic(1, scatteringPdf, 1, lightPdf);

                        lightRayPtr = RayState::Create();
                        auto &lightRay = *lightRayPtr;

                        lightRay.trackRay = rayState.trackRay;
                        lightRay.hop = 0;
                        lightRay.pathHop = rayState.pathHop;
                        lightRay.sample = rayState.sample;
                        lightRay.ray = it.SpawnRay(wi);
                        lightRay.beta = rayState.beta;
                        lightRay.Ld =
                            f * weight / scatteringPdf / lightSelectPdf;
                        lightRay.remainingBounces = rayState.remainingBounces;
                        lightRay.isLightRay = true;
                        lightRay.lightRayInfo.sampledDirection = wi;
                        lightRay.lightRayInfo.sampledLightId = lightNum + 1;
                        lightRay.StartTrace();
                    }
                }
            }
        }
    }

    if (rayState.remainingBounces) {
        Vector3f wo = -rayState.ray.d, wi;
        Float pdf;
        BxDFType flags;
        Spectrum f = it.bsdf->Sample_f(wo, &wi, sampler->Get2D(), &pdf,
                                       BSDF_ALL, &flags);

        if (!f.IsBlack() && pdf > 0.f) {
            bouncePtr = move(rayStatePtr);
            rayStatePtr = nullptr;
            auto &newRay = *bouncePtr;

            newRay.hop = 0;
            newRay.ray = it.SpawnRay(wi);
            newRay.beta *= f * AbsDot(wi, it.shading.n) / pdf;
            newRay.Ld = 0;
            newRay.remainingBounces -= 1;
            newRay.StartTrace();

            // Russian roulette will need etaScale when transmission is
            // supported
            Float rrThreshold = 1.0;
            Spectrum rrBeta = newRay.beta;
            int bounces = maxPathDepth - newRay.remainingBounces - 2;
            if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3) {
                Float q = std::max((Float).05, 1 - rrBeta.MaxComponentValue());
                if (sampler->Get1D() < q) {
                    bouncePtr = nullptr;
                } else {
                    newRay.beta /= 1 - q;
                }
            }
        }
    }

    if (bouncePtr) {
        bouncePtr->sample.dim = sampler->GetCurrentDimension();
        ++nIntersectionTests;
        ++totalRays;
    } else if (shadowRayPtr) {
        /* if bounce isn't produced, this is the last ray in the path */
        shadowRayPtr->remainingBounces = 0;
    }

    if (bouncePtr == nullptr) {
        ReportValue(nRemainingBounces, rayState.remainingBounces);
    }

    return {move(bouncePtr), move(shadowRayPtr), move(lightRayPtr)};
}

void CloudIntegrator::Preprocess(const Scene &scene, Sampler &sampler) {
    bvh = dynamic_pointer_cast<CloudBVH>(scene.aggregate);
    if (bvh == nullptr) {
        throw runtime_error("Top-level primitive must be a CloudBVH");
    }
}

void CloudIntegrator::Render(const Scene &scene) {
    Preprocess(scene, *sampler);
    const Bounds2i sampleBounds = camera->film->GetSampleBounds();
    const Vector2i sampleExtent = sampleBounds.Diagonal();
    unique_ptr<FilmTile> filmTile = camera->film->GetFilmTile(sampleBounds);

    deque<RayStatePtr> rayQueue;
    deque<RayStatePtr> samples;

    /* Generate all the samples */
    size_t i = 0;
    for (Point2i pixel : sampleBounds) {
        sampler->StartPixel(pixel);

        if (!InsideExclusive(pixel, pixelBounds)) continue;

        size_t sample_num = 0;
        do {
            CameraSample cameraSample = sampler->GetCameraSample(pixel);

            RayStatePtr statePtr = RayState::Create();
            auto &state = *statePtr;

            state.sample.id = (pixel.x + pixel.y * sampleExtent.x) *
                                  sampler->samplesPerPixel +
                              sample_num;
            state.sample.dim = sampler->GetCurrentDimension();
            state.sample.pFilm = cameraSample.pFilm;
            state.sample.weight =
                camera->GenerateRayDifferential(cameraSample, &state.ray);
            state.ray.ScaleDifferentials(1 /
                                         sqrt((Float)sampler->samplesPerPixel));
            state.remainingBounces = maxDepth - 1;
            state.StartTrace();

            rayQueue.push_back(move(statePtr));

            ++nIntersectionTests;
            ++nCameraRays;
        } while (sampler->StartNextSample());
    }

    while (not rayQueue.empty()) {
        RayStatePtr statePtr = move(rayQueue.back());
        RayState &state = *statePtr;
        rayQueue.pop_back();

        if (!state.toVisitEmpty()) {
            auto newRayPtr = Trace(move(statePtr), *bvh);
            auto &newRay = *newRayPtr;
            const bool hit = newRay.hit;
            const bool emptyVisit = newRay.toVisitEmpty();

            if (newRay.isShadowRay) {
                if (hit) {
                    newRay.Ld = 0.f;
                    samples.push_back(move(newRayPtr));
                    continue; /* discard */
                } else if (emptyVisit) {
                    samples.push_back(move(newRayPtr));
                } else {
                    rayQueue.push_back(move(newRayPtr));
                }
            } else if (!emptyVisit || hit) {
                rayQueue.push_back(move(newRayPtr));
            } else {
                newRay.Ld = 0.f;
                samples.push_back(move(newRayPtr));
            }
        } else if (state.hit) {
            auto newRays = Shade(move(statePtr), *bvh, scene, sampleExtent,
                                 sampler, maxDepth, arena);

            if (get<0>(newRays)) rayQueue.push_back(move(get<0>(newRays)));
            if (get<1>(newRays)) rayQueue.push_back(move(get<1>(newRays)));
            if (get<2>(newRays)) rayQueue.push_back(move(get<2>(newRays)));
        } else {
            throw runtime_error("unexpected ray state");
        }
    }

    struct CSample {
        Point2f pFilm;
        Spectrum L{0.f};
        Float weight{0.f};
    };

    unordered_map<size_t, CSample> allSamples;

    for (const auto &statePtr : samples) {
        const auto &state = *statePtr;

        if (allSamples.count(state.sample.id) == 0) {
            allSamples[state.sample.id].pFilm = state.sample.pFilm;
            allSamples[state.sample.id].weight = state.sample.weight;
            allSamples[state.sample.id].L = 0.f;
        }

        Spectrum L = state.beta * state.Ld;
        if (L.HasNaNs() || L.y() < -1e-5 || isinf(L.y())) L = Spectrum(0.f);
        allSamples[state.sample.id].L += L;
    }

    cout << allSamples.size() << endl;

    for (const auto &kv : allSamples) {
        filmTile->AddSample(kv.second.pFilm, kv.second.L, kv.second.weight);
    }

    /* Create the final output */
    camera->film->MergeFilmTile(move(filmTile));
    camera->film->WriteImage();
}

CloudIntegrator *CreateCloudIntegrator(const ParamSet &params,
                                       shared_ptr<Sampler> sampler,
                                       shared_ptr<const Camera> camera) {
    const int maxDepth = params.FindOneInt("maxdepth", 5);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    auto globalSampler = dynamic_pointer_cast<GlobalSampler>(sampler);
    if (!globalSampler) {
        throw(runtime_error("CloudIntegrator only supports GlobalSamplers"));
    }

    return new CloudIntegrator(maxDepth, camera, globalSampler, pixelBounds);
}

}  // namespace pbrt
