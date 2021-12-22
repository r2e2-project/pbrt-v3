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
        sceneBase.maxPathDepth = 5;

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
        vector<unique_ptr<CloudBVH>> treelets;
        treelets.resize(sceneBase.GetTreeletCount());

        /* let's load all the treelets */
        for (size_t i = 0; i < treelets.size(); i++) {
            cout << "Loading treelet " << i << "... ";
            treelets[i] = make_unique<CloudBVH>(i, false);
            treelets[i]->LoadTreelet(i);
            cout << "done." << endl;
        }

        while (!rayList.empty()) {
            RayStatePtr theRayPtr = move(rayList.front());
            rayList.pop();

            graphics::ProcessRayOutput processOutput;
            graphics::ProcessRay(move(theRayPtr),
                                 *treelets[theRayPtr->CurrentTreelet()],
                                 sceneBase, arena, processOutput);

            for (auto &r : processOutput.rays) rayList.push(move(r));
            for (auto &s : processOutput.samples) samples.push_back(move(s));
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

        graphics::AccumulateImage(sceneBase.camera, n);
        sceneBase.camera->film->WriteImage();
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
