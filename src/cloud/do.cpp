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

        const string scenePath{argv[1]};
        const string raysPath{argv[2]};

        pbrt::SceneBase scene = pbrt::LoadSceneBase(scenePath, 0);

        /* prepare the scene */
        MemoryArena arena;
        vector<shared_ptr<CloudBVH>> treelets;

        /* let's load all the treelets */
        for (size_t i = 0; i < scene.TreeletCount(); i++) {
            cerr << "Loading treelet " << i << "... ";
            treelets.push_back(pbrt::LoadTreelet(scenePath, i));
            cerr << "done." << endl;
        }

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

        while (!rayList.empty()) {
            RayStatePtr theRayPtr = move(rayList.front());
            rayList.pop();

            ProcessRayOutput processOutput;
            auto &treelet = *treelets[theRayPtr->CurrentTreelet()];
            scene.ProcessRay(move(theRayPtr), treelet, arena, processOutput);

            for (auto &r : processOutput.rays) {
                if (r) rayList.push(move(r));
            }

            if (processOutput.sample) {
                samples.emplace_back(*processOutput.sample);
            }

            if (samples.size() > 1000) {
                scene.AccumulateImage(samples);
                samples.clear();
            }
        }

        if (not samples.empty()) {
            scene.AccumulateImage(samples);
        }

        scene.WriteImage();
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
