#include "expanded.h"

#include <glog/logging.h>

#include "messages/lite.h"

using namespace std;
using namespace pbrt;
using namespace Ptex;

enum class FaceEncoding : int8_t {
    ConstDataPtr,
    Constant,
    Tiled,
    Packed,
    TiledReduced,
    Error,
};

struct FaceData {
    FaceEncoding encoding;
    const void *data_ptr;
    int data_len;
    Res res;

    FaceData(const FaceEncoding encoding, const void *data_ptr,
             const int data_len, const Res &res)
        : encoding(encoding),
          data_ptr(data_ptr),
          data_len(data_len),
          res(res) {}
};

FaceEncoding get_face_encoding(const PtexFaceData *face_data) {
    if (not face_data) {
        throw runtime_error("face_data == nullptr");
    }

    if (dynamic_cast<const PtexReader::ConstDataPtr *>(face_data)) {
        return FaceEncoding::ConstDataPtr;
    }
    if (dynamic_cast<const PtexReader::ConstantFace *>(face_data)) {
        return FaceEncoding::Constant;
    }
    if (dynamic_cast<const PtexReader::TiledFace *>(face_data)) {
        return FaceEncoding::Tiled;
    }
    if (dynamic_cast<const PtexReader::TiledReducedFace *>(face_data)) {
        return FaceEncoding::TiledReduced;
    }
    if (dynamic_cast<const PtexReader::PackedFace *>(face_data)) {
        return FaceEncoding::Packed;
    }
    if (dynamic_cast<const PtexReader::ErrorFace *>(face_data)) {
        throw runtime_error("encountered error face");
    }

    throw runtime_error("unknown face encoding");
}

FaceData get_data(PtexFaceData *raw_data, const int psize) {
    const auto encoding = get_face_encoding(raw_data);

    switch (encoding) {
    case FaceEncoding::ConstDataPtr: {
        auto data = dynamic_cast<PtexReader::ConstDataPtr *>(raw_data);
        return {encoding, data->getData(), psize, raw_data->res()};
    }
    case FaceEncoding::Packed: {
        auto data = dynamic_cast<PtexReader::PackedFace *>(raw_data);
        return {encoding, data->getData(),
                psize * data->res().u() * data->res().v(), raw_data->res()};
    }
    case FaceEncoding::Tiled:
    case FaceEncoding::TiledReduced: {
        throw runtime_error("call the special function for this encoding");
    }
    }

    throw runtime_error("unknown face encoding");
}

template <class T>
std::vector<FaceData> get_tiled_data(PtexFaceData *raw_data, const int psize) {
    std::vector<FaceData> result;

    auto data = dynamic_cast<T *>(raw_data);
    if (!data) {
        throw std::runtime_error("wrong face type passed to get_tiled_data");
    }

    for (auto i = 0; i < data->ntiles(); i++) {
        auto tile_raw_data = data->getTile(i);
        result.push_back(get_data(tile_raw_data, psize));
    }

    return result;
}

ostream &operator<<(ostream &os, const FaceEncoding &fe) {
    switch (fe) {
        // clang-format off
    case FaceEncoding::ConstDataPtr: os << "ConstDataPtr"; break;
    case FaceEncoding::Constant:     os << "Constant"; break;
    case FaceEncoding::Tiled:        os << "Tiled"; break;
    case FaceEncoding::Packed:       os << "Packed"; break;
    case FaceEncoding::Error:        os << "Error"; break;
    case FaceEncoding::TiledReduced: os << "TiledReducedFace"; break;
    default:                         os << "Unknown";
        // clang-format on
    }

    return os;
}

ExpandedPtex::ExpandedPtex(RecordReader *reader) {
    reader->read(&_i);
    reader->read(&_psize);

    _error_pixel.resize(_psize);
    _faceinfo.resize(_i.numFaces);

    reader->read(reinterpret_cast<char *>(_faceinfo.data()),
                 _i.numFaces * sizeof(_faceinfo[0]));

    auto make_face = [this](const FaceEncoding encoding, Res face_res,
                            auto &reader) {
        PtexFaceData *face;
        uint32_t face_data_len;

        switch (encoding) {
        case FaceEncoding::ConstDataPtr:
        case FaceEncoding::Constant:
            face = new ConstantFace(_psize);
            face_data_len = _psize;
            reader->read(reinterpret_cast<char *>(face->getData()),
                         face_data_len);
            break;

        case FaceEncoding::Packed:
            face = new PackedFace(face_res, _psize);
            face_data_len = _psize * face_res.u() * face_res.v();
            reader->read(reinterpret_cast<char *>(face->getData()),
                         face_data_len);
            break;

        default:
            throw runtime_error("make_face only supports simple encodings");
        }

        return face;
    };

    for (int i = 0; i < _i.numFaces; i++) {
        auto &face_info = _faceinfo[i];

        _faces.emplace_back();
        auto &face_all = _faces.back();

        for (int8_t res_u = face_info.res.ulog2; res_u >= 0; res_u--) {
            for (int8_t res_v = face_info.res.vlog2; res_v >= 0; res_v--) {
                FaceEncoding face_encoding;
                Res face_res;
                reader->read(&face_encoding);
                reader->read(&face_res);

                face_all.emplace_back(nullptr);
                auto &face = face_all.back();

                CHECK_EQ(res_u, face_res.ulog2);
                CHECK_EQ(res_v, face_res.vlog2);

                switch (face_encoding) {
                case FaceEncoding::ConstDataPtr:
                case FaceEncoding::Constant:
                case FaceEncoding::Packed:
                    face.reset(make_face(face_encoding, face_res, reader));
                    break;

                case FaceEncoding::Tiled:
                case FaceEncoding::TiledReduced: {
                    Res tile_res;
                    reader->read(&tile_res);

                    const int ntiles_u = face_res.ntilesu(tile_res);
                    const int ntiles_v = face_res.ntilesv(tile_res);
                    const int ntiles = ntiles_u * ntiles_v;

                    face.reset(new TiledFace(face_res, tile_res, _psize));
                    auto *tface = dynamic_cast<TiledFace *>(face.get());

                    for (int i = 0; i < ntiles; i++) {
                        FaceEncoding tface_enc;
                        Res tface_res;
                        reader->read(&tface_enc);
                        reader->read(&tface_res);

                        tface->setTile(make_face(tface_enc, tface_res, reader),
                                       i);
                    }
                }

                case FaceEncoding::Error:
                default:
                    throw runtime_error("unkown face encoding");
                }
            }
        }
    }
}

ExpandedPtex::ExpandedPtex(const std::string &path)
    : ExpandedPtex(make_unique<FileRecordReader>(path).get()) {
    _path = path;
}

ExpandedPtex::ExpandedPtex(const char *data, const size_t data_len)
    : ExpandedPtex(make_unique<LiteRecordReader>(data, data_len).get()) {}

void ExpandedPtex::TiledFace::getPixel(int u, int v, void *result) {
    const int tileu = u >> _tileres.ulog2;
    const int tilev = v >> _tileres.vlog2;
    PtexFaceData *tile = getTile(tilev * _tileres.u() + tileu);
    tile->getPixel(u - (tileu << _tileres.ulog2), v - (tilev << _tileres.vlog2),
                   result);
}

void ExpandedPtex::dump(const std::string &output, PtexReader &ptex) {
    CHECK_NE(ptex.header().meshtype, mt_triangle);
    auto writer = make_unique<LiteRecordWriter>(output);

    const auto num_faces = ptex.numFaces();

    PtexTexture::Info info = ptex.getInfo();
    writer->write(info);
    writer->write<int>(ptex.header().pixelSize());

    vector<FaceInfo> all_faces;
    for (int i = 0; i < num_faces; i++) {
        all_faces.push_back(ptex.getFaceInfo(i));
    }

    writer->write(reinterpret_cast<const char *>(all_faces.data()),
                  all_faces.size() * sizeof(all_faces[0]));

    for (size_t i = 0; i < all_faces.size(); i++) {
        auto &face_info = all_faces[i];

        if (face_info.hasEdits()) {
            throw runtime_error("edited faces are not supported");
        }

        if (face_info.res.ulog2 < 0 or face_info.res.vlog2 < 0) {
            throw runtime_error("unsupported face resolution");
        }

        bool highest_res_processed = false;

        for (int8_t res_u = face_info.res.ulog2; res_u >= 0; res_u--) {
            for (int8_t res_v = face_info.res.vlog2; res_v >= 0; res_v--) {
                Res new_res{res_u, res_v};
                PtexFaceData *raw_data = ptex.getData(i, new_res);
                const auto encoding = get_face_encoding(raw_data);

                if (not highest_res_processed) {
                    CHECK_EQ((encoding == FaceEncoding::Constant ||
                              encoding == FaceEncoding::ConstDataPtr) &&
                                 (res_u != 0 or res_v != 0),
                             false);
                    highest_res_processed = true;
                }

                if (raw_data->res() != new_res) {
                    throw runtime_error("resolution mismatch");
                }

                if (encoding == FaceEncoding::Tiled ||
                    encoding == FaceEncoding::TiledReduced) {
                    auto tiles_data =
                        (encoding == FaceEncoding::Tiled)
                            ? get_tiled_data<PtexReader::TiledFace>(
                                  raw_data, ptex.pixelsize())
                            : get_tiled_data<PtexReader::TiledReducedFace>(
                                  raw_data, ptex.pixelsize());

                    auto base_data =
                        dynamic_cast<PtexReader::TiledFaceBase *>(raw_data);

                    writer->write(encoding);
                    writer->write(raw_data->res());
                    writer->write(base_data->tileRes());

                    for (auto &data : tiles_data) {
                        writer->write(data.encoding);
                        writer->write(data.res);
                        writer->write(
                            reinterpret_cast<const char *>(data.data_ptr),
                            data.data_len);
                    }
                } else {
                    auto data = get_data(raw_data, ptex.pixelsize());
                    writer->write(data.encoding);
                    writer->write(data.res);
                    writer->write(reinterpret_cast<const char *>(data.data_ptr),
                                  data.data_len);
                }
            }
        }
    }

    writer = nullptr;
}

void ExpandedPtex::getData(int faceid, void *buffer, int stride) {
    const auto &fi = getFaceInfo(faceid);
    getData(faceid, buffer, stride, fi.res);
}

void ExpandedPtex::getData(int faceid, void *buffer, int stride, Res res) {
    if (faceid < 0 || faceid > _i.numFaces) {
        PtexUtils::fill(&_error_pixel[0], buffer, stride, res.u(), res.v(),
                        _psize);
        return;
    }

    const int resu = res.u();
    const int resv = res.v();
    const int rowlen = _psize * resu;
    if (stride == 0) stride = rowlen;

    auto face = getData(faceid, res);

    if (face->isConstant()) {
        PtexUtils::fill(face->getData(), buffer, stride, resu, resv, _psize);
    } else if (face->isTiled()) {
        const Res tileres = face->tileRes();
        const int ntilesu = res.ntilesu(tileres);
        const int ntilesv = res.ntilesv(tileres);
        const int tileures = tileres.u();
        const int tilevres = tileres.v();
        const int tilerowlen = _psize * tileures;

        int tile = 0;
        char *dsttilerow = reinterpret_cast<char *>(buffer);
        for (int i = 0; i < ntilesv; i++) {
            char *dsttile = dsttilerow;
            for (int j = 0; j < ntilesu; j++) {
                auto t = face->getTile(tile++);
                if (t->isConstant()) {
                    PtexUtils::fill(t->getData(), dsttile, stride, tileures,
                                    tilevres, _psize);
                } else {
                    PtexUtils::copy(t->getData(), tilerowlen, dsttile, stride,
                                    tilevres, tilerowlen);
                }

                dsttile += tilerowlen;
            }
            dsttilerow += stride * tilevres;
        }
    } else {
        PtexUtils::copy(face->getData(), rowlen, buffer, stride, resv, rowlen);
    }
}

PtexFaceData *ExpandedPtex::getData(int faceid) {
    if (faceid <= 0 || faceid >= _i.numFaces) {
        return new PtexReader::ErrorFace(&_error_pixel[0], _psize, true);
    }

    return _faces[faceid].front().get();
}

PtexFaceData *ExpandedPtex::getData(int faceid, Res res) {
    const auto &fi = _faceinfo[faceid];
    const auto redu = fi.res.ulog2 - res.ulog2;
    const auto redv = fi.res.vlog2 - res.vlog2;

    if (res.ulog2 < 0 || res.vlog2 < 0) {
        throw runtime_error("reductions below 1 pixel not supported");
    } else if (redu < 0 || redv < 0) {
        throw runtime_error("enlargements are not supported");
    } else if (_i.meshType == Ptex::MeshType::mt_triangle) {
        if (redu != redv) {
            throw runtime_error(
                "anisotropic reductions are not supported for triangle mesh");
        }
    }

    // let's compute the index for this reduction
    const auto idx = (fi.res.ulog2 - res.ulog2) * (fi.res.vlog2 + 1) +
                     fi.res.vlog2 - res.vlog2;

    return _faces[faceid].at(idx).get();
}

void ExpandedPtex::getPixel(int faceid, int u, int v, float *result,
                            int firstchan, int nchannels) {
    memset(result, 0, nchannels);
    nchannels = PtexUtils::min(nchannels, _i.numChannels - firstchan);
    if (nchannels <= 0) return;

    auto data = getData(faceid);
    void *pixel = alloca(_psize);
    data->getPixel(u, v, pixel);

    const int datasize = DataSize(_i.dataType);
    if (firstchan) {
        pixel = reinterpret_cast<char *>(pixel) + datasize * firstchan;
    }

    if (_i.dataType == dt_float) {
        memcpy(result, pixel, datasize * nchannels);
    } else {
        ConvertToFloat(result, pixel, _i.dataType, nchannels);
    }
}

void ExpandedPtex::getPixel(int faceid, int u, int v, float *result,
                            int firstchan, int nchannels, Res res) {
    memset(result, 0, nchannels);
    nchannels = PtexUtils::min(nchannels, _i.numChannels - firstchan);
    if (nchannels <= 0) return;

    auto data = getData(faceid, res);
    void *pixel = alloca(_psize);
    data->getPixel(u, v, pixel);

    const int datasize = DataSize(_i.dataType);
    if (firstchan) {
        pixel = reinterpret_cast<char *>(pixel) + datasize * firstchan;
    }

    if (_i.dataType == dt_float) {
        memcpy(result, pixel, datasize * nchannels);
    } else {
        ConvertToFloat(result, pixel, _i.dataType, nchannels);
    }
}

template <typename T>
ExpandedPtexTexture<T>::ExpandedPtexTexture(const string &filename, Float gamma)
    : texture(new ExpandedPtex{filename}), gamma(gamma) {}

template <typename T>
inline T fromResult(int nc, float *result) {
    return T::unimplemented;
}

template <>
inline Float fromResult<Float>(int nc, float *result) {
    if (nc == 1)
        return result[0];
    else
        return (result[0] + result[1] + result[2]) / 3;
}

template <>
inline Spectrum fromResult<Spectrum>(int nc, float *result) {
    if (nc == 1)
        return Spectrum(result[0]);
    else {
        Float rgb[3] = {result[0], result[1], result[2]};
        return Spectrum::FromRGB(rgb);
    }
}

template <typename T>
T ExpandedPtexTexture<T>::Evaluate(const SurfaceInteraction &si) const {
    Ptex::PtexPtr<Ptex::PtexFilter> filter{Ptex::PtexFilter::getFilter(
        texture.get(), {Ptex::PtexFilter::FilterType::f_bspline})};

    const int nc = texture->numChannels();

    float result[3];
    int first_chan = 0;
    filter->eval(result, first_chan, nc, si.faceIndex, si.uv[0], si.uv[1],
                 si.dudx, si.dvdx, si.dudy, si.dvdy);
    filter->release();

    if (gamma != 1) {
        for (int i = 0; i < nc; ++i) {
            if (result[i] >= 0 && result[i] <= 1) {
                // FIXME: should use something more efficient here
                result[i] = std::pow(result[i], gamma);
            }
        }
    }

    return fromResult<T>(nc, result);
}

ExpandedPtexTexture<Float> *pbrt::CreateExpandedPtexFloatTexture(
    const Transform &tex2world, const TextureParams &tp) {
    std::string filename = tp.FindFilename("filename");
    Float gamma = tp.FindFloat("gamma", 2.2);
    return new ExpandedPtexTexture<Float>(filename, gamma);
}

ExpandedPtexTexture<Spectrum> *pbrt::CreateExpandedPtexSpectrumTexture(
    const Transform &tex2world, const TextureParams &tp) {
    std::string filename = tp.FindFilename("filename");
    Float gamma = tp.FindFloat("gamma", 2.2);
    return new ExpandedPtexTexture<Spectrum>(filename, gamma);
}
