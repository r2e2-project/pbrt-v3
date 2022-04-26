#ifndef PBRT_SRC_CLOUD_PTEX_EXPANDED_H
#define PBRT_SRC_CLOUD_PTEX_EXPANDED_H

#include <utility>
#include <vector>

// clang-format off
#include <iostream>
#include <PtexReader.h>
#include <Ptexture.h>
// clang-format on

namespace pbrt {

namespace ptex::util {

enum class FaceEncoding {
    ConstDataPtr,
    Constant,
    Tiled,
    Packed,
    TiledReduced,
    Error,
};

FaceEncoding get_face_encoding(const PtexFaceData *face_data);

std::pair<void *, int> get_data(PtexFaceData *raw_data, const int psize);

template <class T>
std::vector<std::pair<void *, int>> get_tiled_data(PtexFaceData *raw_data,
                                                   const int psize) {
    std::vector<std::pair<void *, int>> result;

    auto data = dynamic_cast<T *>(raw_data);
    for (auto i = 0; i < data->ntiles(); i++) {
        auto tile_raw_data = data->getTile(i);
        result.push_back(get_data(tile_raw_data, psize));
    }

    return result;
}

}  // namespace ptex::util

class ExpandedPtex : public PtexTexture {};

}  // namespace pbrt

#endif /* PBRT_SRC_CLOUD_PTEX_EXPANDED_H */
