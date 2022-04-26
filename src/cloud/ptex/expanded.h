#ifndef PBRT_SRC_CLOUD_PTEX_EXPANDED_H
#define PBRT_SRC_CLOUD_PTEX_EXPANDED_H

// clang-format off
#include <iostream>
#include <PtexReader.h>
#include <Ptexture.h>
// clang-format on

namespace pbrt {

class ExpandedPtex : public Ptex::PtexReader {
  public:
    enum class FaceEncoding {
        ConstDataPtr,
        Constant,
        Tiled,
        Packed,
        TiledReduced,
        Error,
    };

    static FaceEncoding get_face_encoding(const PtexFaceData *face_data);

    static std::pair<void *, int> get_data(PtexFaceData *raw_data,
                                           const int psize);

    static std::vector<std::pair<void *, int>> get_tiled_data(
        PtexFaceData *raw_data, const int psize);

    static std::vector<std::pair<void *, int>> get_reduced_tile_data(
        PtexFaceData *raw_data, const int psize);
};

}  // namespace pbrt

#endif /* PBRT_SRC_CLOUD_PTEX_EXPANDED_H */
