#include <glog/logging.h>

#include <memory>

// clang-format off
#include <iostream>
#include <PtexReader.h>
// clang-format on

#include "cloud/ptex/expanded.h"
#include "messages/lite.h"
#include "util/path.h"
#include "util/util.h"

using namespace std;
using namespace pbrt;

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " INPUT OUTPUT" << endl;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        usage((argc <= 0) ? "ptextract" : argv[0]);
        return EXIT_FAILURE;
    }

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_minloglevel = 0;  // INFO
    FLAGS_colorlogtostderr = true;

    const string ptex_file{argv[1]};
    const string output_file{argv[2]};

    Ptex::String error;
    PtexPtr<Ptex::PtexReader> ptex_texture{
        new Ptex::PtexReader(false, nullptr, nullptr)};
    if (!ptex_texture->open(ptex_file.c_str(), error)) {
        throw runtime_error("cannot open the texture: " + ptex_file +
                            " (reason: " + error + ")");
    }

    const auto num_faces = ptex_texture->numFaces();

    LOG(WARNING) << boolalpha << endl
                 << "* Texture info: " << endl
                 << "  - Size      = "
                 << pbrt::format_bytes(pbrt::roost::file_size(ptex_file))
                 << endl
                 << "  - Faces     = " << num_faces << endl
                 << "  - Mesh type = "
                 << Ptex::MeshTypeName(ptex_texture->meshType()) << endl
                 << "  - Mipmaps   = " << ptex_texture->hasMipMaps() << endl
                 << "  - Edits     = " << ptex_texture->hasEdits();

    ExpandedPtex::dump(output_file, *ptex_texture);

    LOG(WARNING) << "* Expanded texture size = "
                 << pbrt::format_bytes(pbrt::roost::file_size(output_file));

    return EXIT_SUCCESS;
}
