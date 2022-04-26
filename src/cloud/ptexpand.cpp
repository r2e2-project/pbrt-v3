#include <glog/logging.h>

#include <iostream>

#include "cloud/ptex/expanded.h"
#include "util/path.h"
#include "util/util.h"

using namespace std;
using namespace pbrt;
using namespace pbrt::ptex::util;

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " INPUT" << endl;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        usage((argc <= 0) ? "ptextract" : argv[0]);
        return EXIT_FAILURE;
    }

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_minloglevel = 0;  // INFO
    FLAGS_colorlogtostderr = true;

    const string ptex_file{argv[1]};

    Ptex::String error;
    PtexPtr<Ptex::PtexReader> ptex_texture{
        new Ptex::PtexReader(false, nullptr, nullptr)};
    if (!ptex_texture->open(ptex_file.c_str(), error)) {
        throw runtime_error("cannot open the texture: " + ptex_file +
                            " (reason: " + error + ")");
    }

    const auto num_faces = ptex_texture->numFaces();

    LOG(WARNING) << "* Texture info: " << endl
                 << "  - Size      = "
                 << pbrt::format_bytes(pbrt::roost::file_size(ptex_file))
                 << endl
                 << "  - Faces     = " << num_faces << endl
                 << "  - Mesh type = "
                 << ((ptex_texture->header().meshtype == Ptex::mt_triangle)
                         ? "triangle"
                         : "quad");

    CHECK_NE(ptex_texture->header().meshtype, Ptex::mt_triangle);

    size_t expanded_size = 0;

    for (int i = 0; i < num_faces; i++) {
        auto &face_info = ptex_texture->getFaceInfo(i);

        LOG_EVERY_N(INFO, 1000)
            << "[F" << i << "] res=" << static_cast<int>(face_info.res.u())
            << "x" << static_cast<int>(face_info.res.v())
            << ", logres=" << static_cast<int>(face_info.res.ulog2) << "x"
            << static_cast<int>(face_info.res.vlog2) << boolalpha
            << ", edits=" << face_info.hasEdits()
            << ", subface=" << face_info.isSubface();

        if (face_info.hasEdits()) {
            throw runtime_error("edited faces are not supported");
        }

        if (face_info.res.ulog2 < 0 or face_info.res.vlog2 < 0) {
            throw runtime_error("unsupported face resolution");
        }

        for (int8_t res_u = face_info.res.ulog2; res_u >= 0; res_u--) {
            for (int8_t res_v = face_info.res.vlog2; res_v >= 0; res_v--) {
                Ptex::Res new_res{res_u, res_v};
                PtexFaceData *raw_data = ptex_texture->getData(i, new_res);
                const auto encoding = get_face_encoding(raw_data);

                if (encoding == FaceEncoding::Tiled ||
                    encoding == FaceEncoding::TiledReduced) {
                    auto tiles_data =
                        (encoding == FaceEncoding::Tiled)
                            ? get_tiled_data<Ptex::PtexReader::TiledFace>(
                                  raw_data, ptex_texture->pixelsize())
                            : get_tiled_data<
                                  Ptex::PtexReader::TiledReducedFace>(
                                  raw_data, ptex_texture->pixelsize());

                    size_t total_len = 0;
                    for (auto &d : tiles_data) total_len += d.second;

                    expanded_size += total_len;
                } else {
                    auto data = get_data(raw_data, ptex_texture->pixelsize());

                    expanded_size += data.second;
                }
            }
        }
    }

    LOG(INFO) << "Expanded size = " << format_bytes(expanded_size) << endl;

    return EXIT_SUCCESS;
}
