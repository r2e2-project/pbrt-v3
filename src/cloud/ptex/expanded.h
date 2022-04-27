#ifndef PBRT_SRC_CLOUD_PTEX_EXPANDED_H
#define PBRT_SRC_CLOUD_PTEX_EXPANDED_H

#include <Ptexture.h>

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace pbrt {

namespace ptex::util {

enum class FaceEncoding : int8_t {
    ConstDataPtr,
    Constant,
    Tiled,
    Packed,
    TiledReduced,
    Error,
};

FaceEncoding get_face_encoding(const PtexFaceData* face_data);

std::tuple<FaceEncoding, void*, int> get_data(PtexFaceData* raw_data,
                                              const int psize);

template <class T>
std::vector<std::tuple<FaceEncoding, void*, int>> get_tiled_data(
    PtexFaceData* raw_data, const int psize) {
    std::vector<std::tuple<FaceEncoding, void*, int>> result;

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

  private:
    class PackedFace : public PtexFaceData {
      private:
        Ptex::Res _res;
        int _psize;
        std::unique_ptr<char[]> _data;

      public:
        PackedFace(Ptex::Res res, int psize)
            : _res(res),
              _psize(psize),
              _data(std::make_unique<char[]>(_res.u() * _res.v() * _psize)) {}

        virtual void release() {}
        virtual bool isConstant() { return false; }
        virtual Ptex::Res res() { return _res; }
        virtual bool isTiled() { return false; }
        virtual Ptex::Res tileRes() { return 0; }
        virtual PtexFaceData* getTile(int tile) { return nullptr; }

        virtual void* getData() { return reinterpret_cast<void*>(_data.get()); }
        virtual void getPixel(int u, int v, void* result) {
            memcpy(result, _data.get() + (v * _res.u() + u) * _psize, _psize);
        }
    };

    class ConstantFace : public PackedFace {
      public:
        ConstantFace(int psize) : PackedFace(0, psize) {}
        virtual bool isConstant() { return true; }
    };

    class TiledFace : public PtexFaceData {
      private:
        Ptex::Res _res;
        Ptex::Res _tileres;
        std::vector<PtexPtr<PtexFaceData>> _tiles;

      public:
        TiledFace(Ptex::Res res, Ptex::Res tileres)
            : _res(res),
              _tileres(tileres),
              _tiles(_tileres.u() * _tileres.v()) {}

        virtual void release() {}
        virtual bool isConstant() { return false; }
        virtual Ptex::Res res() { return _res; }
        virtual bool isTiled() { return true; }
        virtual Ptex::Res tileRes() { return _tileres; }
        virtual PtexFaceData* getTile(int t) { return _tiles.at(t).get(); }

        void setTile(PtexPtr<PtexFaceData>& tile, int t) {
            _tiles[t].swap(tile);
        }

        virtual void* getData() { return nullptr; }
        virtual void getPixel(int u, int v, void* result);
    };

  public:
    ExpandedPtex(const std::string& path, const char* data,
                 const size_t data_len);

    virtual void release() { delete this; }
    virtual const char* path() { return _path.c_str(); }
    virtual Info getInfo() { return _info; }

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

    Info _info;
    std::vector<Ptex::FaceInfo> _faceinfo;
    std::vector<std::map<Ptex::Res, PtexPtr<PtexFaceData>>> _faces;
};

}  // namespace pbrt

#endif /* PBRT_SRC_CLOUD_PTEX_EXPANDED_H */
