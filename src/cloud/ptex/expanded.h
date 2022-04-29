#ifndef PBRT_SRC_CLOUD_PTEX_EXPANDED_H
#define PBRT_SRC_CLOUD_PTEX_EXPANDED_H

#include <Ptexture.h>

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

// clang-format off
#include <iostream>
#include <PtexReader.h>
// clang-format on

#include "interaction.h"
#include "paramset.h"
#include "pbrt.h"
#include "texture.h"

class RecordReader;

namespace pbrt {

class ExpandedPtex : public Ptex::PtexTexture {
  protected:
    virtual ~ExpandedPtex() {}

  private:
    struct FaceDeleter {
        void operator()(PtexFaceData* b) { b->release(); }
    };

    class ConstPtrFace : public PtexFaceData {
      protected:
        void* _data;
        int _pixelsize;

      public:
        ConstPtrFace(void* data, int pixelsize)
            : _data(data), _pixelsize(pixelsize) {}

        virtual void release() {}  // memory is managed by ExpandedPtex
        virtual Ptex::Res res() { return 0; }
        virtual bool isConstant() { return true; }
        virtual void* getData() { return _data; }
        virtual bool isTiled() { return false; }
        virtual Ptex::Res tileRes() { return 0; }
        virtual PtexFaceData* getTile(int) { return 0; }
        virtual void getPixel(int, int, void* result) {
            memcpy(result, _data, _pixelsize);
        }
    };

    class PackedFace : public PtexFaceData {
      protected:
        const Ptex::Res _res;
        const int _psize;
        std::unique_ptr<char[]> _data;

      public:
        PackedFace(Ptex::Res res, int psize)
            : _res(res),
              _psize(psize),
              _data(std::make_unique<char[]>(_res.u() * _res.v() * _psize)) {}

        virtual void release() {}  // memory is managed by ExpandedPtex
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

        virtual void getPixel(int, int, void* result) {
            memcpy(result, _data.get(), _psize);
        }
    };

    class TiledFace : public PtexFaceData {
      private:
        const Ptex::Res _res;
        const Ptex::Res _tileres;
        const int _psize;
        std::vector<std::unique_ptr<PtexFaceData, FaceDeleter>> _tiles;

      public:
        TiledFace(Ptex::Res res, Ptex::Res tileres, int psize)
            : _res(res),
              _tileres(tileres),
              _psize(psize),
              _tiles(_tileres.u() * _tileres.v()) {}

        virtual void release() {}  // memory is managed by ExpandedPtex
        virtual bool isConstant() { return false; }
        virtual Ptex::Res res() { return _res; }
        virtual bool isTiled() { return true; }
        virtual Ptex::Res tileRes() { return _tileres; }
        virtual PtexFaceData* getTile(int t) { return _tiles.at(t).get(); }
        void setTile(PtexFaceData* tile, int t) { _tiles[t].reset(tile); }

        virtual void* getData() { return nullptr; }
        virtual void getPixel(int u, int v, void* result);
    };

    ExpandedPtex(RecordReader* reader);

  public:
    ExpandedPtex(const std::string& path);
    ExpandedPtex(const char* data, const size_t data_len);

    static void dump(const std::string& output, Ptex::PtexReader& ptex);

    virtual void release() { delete this; }
    virtual const char* path() { return _path.c_str(); }
    virtual Info getInfo() { return _i; }

    virtual Ptex::MeshType meshType() { return _i.meshType; }
    virtual Ptex::DataType dataType() { return _i.dataType; }
    virtual Ptex::BorderMode uBorderMode() { return _i.uBorderMode; }
    virtual Ptex::BorderMode vBorderMode() { return _i.vBorderMode; }
    virtual Ptex::EdgeFilterMode edgeFilterMode() { return _i.edgeFilterMode; }
    virtual int alphaChannel() { return _i.alphaChannel; }
    virtual int numChannels() { return _i.numChannels; }
    virtual int numFaces() { return _i.numFaces; }
    virtual bool hasEdits() { return false; }
    virtual bool hasMipMaps() { return false; }

    virtual PtexMetaData* getMetaData() { return nullptr; };

    virtual const Ptex::FaceInfo& getFaceInfo(int i) { return _faceinfo[i]; }

    virtual void getData(int faceid, void* buffer, int stride);
    virtual void getData(int faceid, void* buffer, int stride, Ptex::Res res);

    virtual PtexFaceData* getData(int faceid);
    virtual PtexFaceData* getData(int faceid, Ptex::Res res);

    virtual void getPixel(int faceid, int u, int v, float* result,
                          int firstchan, int nchannels);
    virtual void getPixel(int faceid, int u, int v, float* result,
                          int firstchan, int nchannels, Ptex::Res res);

  private:
    std::string _path{};

    Info _i{};
    int _psize{};
    std::vector<Ptex::FaceInfo> _faceinfo{};
    std::vector<uint8_t> _const_data{};
    std::vector<std::vector<std::unique_ptr<PtexFaceData, FaceDeleter>>>
        _faces{};

    std::vector<char> _error_pixel{};
};

// PtexTexture Declarations
template <typename T>
class ExpandedPtexTexture : public Texture<T> {
  public:
    // PtexTexture Public Methods
    ExpandedPtexTexture(const std::string& filename, Float gamma);
    ~ExpandedPtexTexture() {}
    T Evaluate(const SurfaceInteraction&) const;

    TextureType GetType() const { return TextureType::ExpandedPtex; }

  private:
    struct TextureDeleter {
        void operator()(ExpandedPtex* b) { b->release(); }
    };

    const std::unique_ptr<ExpandedPtex, TextureDeleter> texture;
    const Float gamma;
};

ExpandedPtexTexture<Float>* CreateExpandedPtexFloatTexture(
    const Transform& tex2world, const TextureParams& tp);
ExpandedPtexTexture<Spectrum>* CreateExpandedPtexSpectrumTexture(
    const Transform& tex2world, const TextureParams& tp);

}  // namespace pbrt

#endif /* PBRT_SRC_CLOUD_PTEX_EXPANDED_H */
