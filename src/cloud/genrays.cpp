#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "core/camera.h"
#include "core/geometry.h"
#include "core/transform.h"
#include "messages/utils.h"
#include "pbrt/main.h"
#include "util/exception.h"

using namespace std;
using namespace pbrt;

void usage(const char *argv0) {
    cerr << argv0 << " SCENE-DATA OUTPUT [SPP]" << endl;
}

int main(int argc, char const *argv[]) {
    try {
        if (argc <= 0) {
            abort();
        }

        if (argc < 3) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        const string scenePath{argv[1]};
        const string outputPath{argv[2]};
        int spp = 0;

        if (argc == 4) {
            spp = stoi(argv[3]);
        }

        pbrt::SceneBase scene = pbrt::LoadSceneBase(scenePath, spp);
        scene.SetPathDepth(5);

        /* Generate all the samples */
        protobuf::RecordWriter rayWriter{outputPath};
        size_t sampleCount = 0;

        char rayBuffer[sizeof(RayState)];

        for (size_t sample = 0; sample < scene.SamplesPerPixel(); sample++) {
            for (Point2i pixel : scene.SampleBounds()) {
                auto ray = scene.GenerateCameraRay(pixel, sample);
                const auto len = ray->Serialize(rayBuffer);
                rayWriter.write(rayBuffer + 4, len - 4);
                sampleCount++;
            }
        }

        cerr << sampleCount << " sample(s) were generated and written to "
             << outputPath << endl;
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
