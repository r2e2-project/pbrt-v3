#include <glog/logging.h>

#include <iostream>

#include "cloud/ptex/expanded.h"
#include "util/util.h"

using namespace std;
using namespace pbrt;

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

    const string ptex_file{argv[1]};

    Ptex::String error;
    PtexPtr<Ptex::PtexReader> ptex_texture{
        new Ptex::PtexReader(false, nullptr, nullptr)};
    if (!ptex_texture->open(ptex_file.c_str(), error)) {
        throw runtime_error("cannot open the texture: " + ptex_file +
                            " (reason: " + error + ")");
    }

    const auto num_faces = ptex_texture->numFaces();

    LOG(INFO) << "* Texture info: " << endl
              << "  - Faces = " << num_faces << endl;

    size_t expanded_size = 0;

    for (int i = 0; i < num_faces; i++) {
        auto &face_info = ptex_texture->getFaceInfo(i);

        LOG_EVERY_N(INFO, 100)
            << "[F" << i << "] res=" << static_cast<int>(face_info.res.u())
            << "x" << static_cast<int>(face_info.res.v())
            << ", logres=" << static_cast<int>(face_info.res.ulog2) << "x"
            << static_cast<int>(face_info.res.vlog2) << boolalpha
            << ", edits=" << face_info.hasEdits()
            << ", subface=" << face_info.isSubface();

        if (face_info.res.ulog2 < 0 or face_info.res.vlog2 < 0) {
            throw runtime_error("unsupported face resolution");
        }

        for (int8_t res_u = face_info.res.ulog2; res_u >= 0; res_u--) {
            for (int8_t res_v = face_info.res.vlog2; res_v >= 0; res_v--) {
                Ptex::Res new_res{res_u, res_v};
                PtexFaceData *raw_data = ptex_texture->getData(i, new_res);
                const auto encoding = ExpandedPtex::get_face_encoding(raw_data);

                if (encoding == ExpandedPtex::FaceEncoding::Tiled ||
                    encoding == ExpandedPtex::FaceEncoding::TiledReduced) {
                    auto data = (encoding == ExpandedPtex::FaceEncoding::Tiled)
                                    ? ExpandedPtex::get_tiled_data(
                                          raw_data, ptex_texture->pixelsize())
                                    : ExpandedPtex::get_reduced_tile_data(
                                          raw_data, ptex_texture->pixelsize());

                    size_t total_len = 0;
                    for (auto &d : data) total_len += d.second;

                    expanded_size += total_len;
                } else {
                    auto data = ExpandedPtex::get_data(
                        raw_data, ptex_texture->pixelsize());

                    expanded_size += data.second;
                }
            }
        }
    }

    LOG(INFO) << "Expanded size = " << format_bytes(expanded_size) << endl;

    return EXIT_SUCCESS;
}
