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

FaceEncoding get_face_encoding(const PtexFaceData* face_data);

std::pair<void*, int> get_data(PtexFaceData* raw_data, const int psize);

template <class T>
std::vector<std::pair<void*, int>> get_tiled_data(PtexFaceData* raw_data,
                                                  const int psize) {
    std::vector<std::pair<void*, int>> result;

    auto data = dynamic_cast<T*>(raw_data);
    if (!data) {
        throw std::runtime_error("wrong face type passed to get_tiled_data");
    }

    for (auto i = 0; i < data->ntiles(); i++) {
        auto tile_raw_data = data->getTile(i);
        result.push_back(get_data(tile_raw_data, psize));
    }

    return result;
}

}  // namespace ptex::util

class ExpandedPtex : public PtexTexture {
  protected:
    virtual ~ExpandedPtex() {}

  public:
    ExpandedPtex(const std::string& path) : _path(path) {}

    virtual void release() { delete this; }
    virtual const char* path() { return _path.c_str(); }
    virtual Info getInfo() = 0;

    virtual Ptex::MeshType meshType() = 0;
    virtual Ptex::DataType dataType() = 0;
    virtual Ptex::BorderMode uBorderMode() = 0;
    virtual Ptex::BorderMode vBorderMode() = 0;
    virtual Ptex::EdgeFilterMode edgeFilterMode() = 0;

    virtual int alphaChannel() = 0;
    virtual int numChannels() = 0;
    virtual int numFaces() = 0;
    virtual bool hasEdits() = 0;

    virtual bool hasMipMaps() = 0;

    virtual PtexMetaData* getMetaData() = 0;

    virtual const Ptex::FaceInfo& getFaceInfo(int faceid) = 0;

    virtual void getData(int faceid, void* buffer, int stride) = 0;
    virtual void getData(int faceid, void* buffer, int stride,
                         Ptex::Res res) = 0;

    virtual PtexFaceData* getData(int faceid) = 0;
    virtual PtexFaceData* getData(int faceid, Ptex::Res res) = 0;

    virtual void getPixel(int faceid, int u, int v, float* result,
                          int firstchan, int nchannels) = 0;
    virtual void getPixel(int faceid, int u, int v, float* result,
                          int firstchan, int nchannels, Ptex::Res res) = 0;

  private:
    std::string _path;
};

}  // namespace pbrt

#endif /* PBRT_SRC_CLOUD_PTEX_EXPANDED_H */
