#include "pimage.h"

#include <iostream>
#include <memory>
#include <string>

#include "imageio.h"

using namespace std;
using namespace pbrt;

int main(int argc, char const *argv[]) {
    if (argc != 3) {
        cerr << "Usage: pimage_test PNG_FILE PARTITIONS" << endl;
        return EXIT_FAILURE;
    }

    string filename{argv[1]};
    size_t partitions = stoull(argv[2]);

    Point2i resolution;
    unique_ptr<pbrt::RGBSpectrum[]> texels{nullptr};
    texels = ReadImage(filename, &resolution);

    pbrt::PartitionedImage img{resolution, texels.get(), partitions,
                               pbrt::ImageWrap::Clamp};

    unique_ptr<pbrt::RGBSpectrum[]> output =
        make_unique<RGBSpectrum[]>(512 * 512);

    for (size_t i = 0; i < 512; i++) {
        for (size_t j = 0; j < 512; j++) {
            output[i + j * 512] = img.Lookup(Point2f{i / 512.f, j / 512.f});
        }
    }

    pbrt::WriteImage("output.png", (Float *)output.get(),
                     {Point2i(0, 0), Point2i(512, 512)}, Point2i(512, 512));

    return 0;
}
