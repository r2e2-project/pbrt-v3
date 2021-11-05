#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "accelerators/cloud.h"
#include "cloud/manager.h"
#include "messages/serialization.h"
#include "messages/utils.h"
#include "pbrt/main.h"
#include "pbrt/raystate.h"
#include "util/exception.h"

using namespace std;
using namespace pbrt;

void usage(const char *argv0) {
    cerr << argv0 << " SCENE-DATA CAMERA-RAYS" << endl;
}

enum class Operation { Trace, Shade };

int main(int argc, char const *argv[]) {
    try {
        if (argc <= 0) {
            abort();
        }

        if (argc != 3) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        FLAGS_log_prefix = false;
        google::InitGoogleLogging(argv[0]);

        /* CloudBVH requires this */
        PbrtOptions.nThreads = 1;

        const string scenePath{argv[1]};
        const string raysPath{argv[2]};

        pbrt::scene::Base sceneBase = pbrt::scene::LoadBase(scenePath, 0);

        queue<RayStatePtr> rayList;
        vector<Sample> samples;

        /* loading all the rays */
        {
            protobuf::RecordReader reader{raysPath};
            while (!reader.eof()) {
                string rayStr;

                if (reader.read(&rayStr)) {
                    auto rayStatePtr = RayState::Create();
                    rayStatePtr->Deserialize(rayStr.data(), rayStr.length());
                    rayList.push(move(rayStatePtr));
                }
            }
        }

        cerr << rayList.size() << " RayState(s) loaded." << endl;

        if (!rayList.size()) {
            return EXIT_SUCCESS;
        }

        /* prepare the scene */
        MemoryArena arena;
        auto &camera = sceneBase.camera;
        auto &sampler = sceneBase.sampler;
        auto &fakeScene = sceneBase.fakeScene;

        vector<unique_ptr<CloudBVH>> treelets;
        treelets.resize(sceneBase.GetTreeletCount());

        /* let's load all the treelets */
        for (size_t i = 0; i < treelets.size(); i++) {
            cout << "Loading treelet " << i << "... ";
            treelets[i] = make_unique<CloudBVH>(i, false);
            treelets[i]->LoadTreelet(i);
            cout << "done." << endl;
        }

        const auto sampleExtent = camera->film->GetSampleBounds().Diagonal();
        const int maxDepth = 5;

        while (!rayList.empty()) {
            RayStatePtr theRayPtr = move(rayList.front());
            RayState &theRay = *theRayPtr;
            rayList.pop();

            const TreeletId rayTreeletId = theRay.CurrentTreelet();

            if (!theRay.toVisitEmpty()) {
                auto newRayPtr = graphics::TraceRay(move(theRayPtr),
                                                    *treelets[rayTreeletId]);
                auto &newRay = *newRayPtr;

                const bool hit = newRay.HasHit();
                const bool emptyVisit = newRay.toVisitEmpty();

                if (newRay.IsShadowRay()) {
                    if (hit || emptyVisit) {
                        newRay.Ld = hit ? 0.f : newRay.Ld;
                        samples.emplace_back(*newRayPtr);
                    } else {
                        rayList.push(move(newRayPtr));
                    }
                } else if (newRay.IsLightRay()) {
                    if (emptyVisit) {
                        Spectrum Li{0.f};

                        const auto sLight = newRay.lightRayInfo.sampledLightId;

                        if (newRay.HasHit()) {
                            const auto aLight = newRay.hitInfo.arealight;
                            if (aLight == sLight) {
                                Li = dynamic_pointer_cast<AreaLight>(
                                         fakeScene->lights[aLight - 1])
                                         ->L(newRay.hitInfo.isect,
                                             -newRay.lightRayInfo
                                                  .sampledDirection);
                            }
                        } else {
                            Li = fakeScene->lights[sLight - 1]->Le(newRay.ray);
                        }

                        if (!Li.IsBlack()) {
                            newRay.Ld *= Li;
                            samples.emplace_back(*newRayPtr);
                        }
                    } else {
                        rayList.push(move(newRayPtr));
                    }
                } else if (!emptyVisit || hit) {
                    rayList.push(move(newRayPtr));
                } else if (emptyVisit) {
                    newRay.Ld = 0.f;
                    if (newRay.remainingBounces == maxDepth - 1) {
                        for (const auto &light : fakeScene->infiniteLights) {
                            newRay.Ld += light->Le(newRay.ray);
                        }
                    }
                    samples.emplace_back(*newRayPtr);
                }
            } else if (theRay.HasHit()) {
                RayStatePtr bounceRay, shadowRay, lightRay;
                tie(bounceRay, shadowRay, lightRay) = graphics::ShadeRay(
                    move(theRayPtr), *treelets[rayTreeletId], *fakeScene,
                    sampleExtent, sampler, maxDepth, arena);

                if (bounceRay != nullptr) {
                    rayList.push(move(bounceRay));
                }

                if (shadowRay != nullptr) {
                    rayList.push(move(shadowRay));
                }

                if (lightRay != nullptr) {
                    rayList.push(move(lightRay));
                }
            }
        }

        map<pair<float, float>, pbrt::Sample> newSamples;

        for (auto &s : samples) {
            const auto key = make_pair(s.pFilm.x, s.pFilm.y);
            if (newSamples.count(key)) {
                newSamples[make_pair(s.pFilm.x, s.pFilm.y)].L += s.L;
            } else {
                newSamples.emplace(make_pair(s.pFilm.x, s.pFilm.y), move(s));
            }
        }

        vector<Sample> n;
        for (auto &s : newSamples) {
            n.push_back(move(s.second));
        }

        graphics::AccumulateImage(camera, n);
        camera->film->WriteImage();
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
