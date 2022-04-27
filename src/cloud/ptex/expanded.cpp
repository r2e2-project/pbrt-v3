#include "expanded.h"

#include <glog/logging.h>

// clang-format off
#include <iostream>
#include <PtexReader.h>
// clang-format on

#include "messages/lite.h"

using namespace std;
using namespace pbrt;

using FaceEncoding = ptex::util::FaceEncoding;

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

FaceEncoding ptex::util::get_face_encoding(const PtexFaceData *face_data) {
    if (not face_data) {
        throw runtime_error("face_data == nullptr");
    }

    if (dynamic_cast<const Ptex::PtexReader::ConstDataPtr *>(face_data)) {
        return FaceEncoding::ConstDataPtr;
    }
    if (dynamic_cast<const Ptex::PtexReader::ConstantFace *>(face_data)) {
        return FaceEncoding::Constant;
    }
    if (dynamic_cast<const Ptex::PtexReader::TiledFace *>(face_data)) {
        return FaceEncoding::Tiled;
    }
    if (dynamic_cast<const Ptex::PtexReader::TiledReducedFace *>(face_data)) {
        return FaceEncoding::TiledReduced;
    }
    if (dynamic_cast<const Ptex::PtexReader::PackedFace *>(face_data)) {
        return FaceEncoding::Packed;
    }
    if (dynamic_cast<const Ptex::PtexReader::ErrorFace *>(face_data)) {
        throw runtime_error("encountered error face");
    }

    throw runtime_error("unknown face encoding");
}

ptex::util::FaceData ptex::util::get_data(PtexFaceData *raw_data,
                                          const int psize) {
    const auto encoding = get_face_encoding(raw_data);

    switch (encoding) {
    case FaceEncoding::ConstDataPtr: {
        auto data = dynamic_cast<Ptex::PtexReader::ConstDataPtr *>(raw_data);
        return {encoding, data->getData(), psize, raw_data->res()};
    }
    case FaceEncoding::Packed: {
        auto data = dynamic_cast<Ptex::PtexReader::PackedFace *>(raw_data);
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

ExpandedPtex::ExpandedPtex(const string &path, const char *texture_data,
                           const size_t texture_data_len)
    : _path(path) {
    LiteRecordReader reader{texture_data, texture_data_len};
    reader.read(&_i);
    reader.read(&_psize);

    _faceinfo.resize(_i.numFaces);
    reader.read(reinterpret_cast<char *>(_faceinfo.data()),
                _i.numFaces * sizeof(_faceinfo[0]));

    auto make_face = [this](const FaceEncoding encoding, Ptex::Res face_res,
                            auto &reader) {
        PtexFaceData *face;
        uint32_t face_data_len;

        switch (encoding) {
        case FaceEncoding::ConstDataPtr:
        case FaceEncoding::Constant:
            face = new ConstantFace(_psize);
            face_data_len = _psize;
            reader.read(reinterpret_cast<char *>(face->getData()),
                        face_data_len);
            break;

        case FaceEncoding::Packed:
            face = new PackedFace(face_res, _psize);
            face_data_len = _psize * face_res.u() * face_res.v();
            reader.read(reinterpret_cast<char *>(face->getData()),
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
                Ptex::Res face_res;
                reader.read(&face_encoding);
                reader.read(&face_res);

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
                    Ptex::Res tile_res;
                    reader.read(&tile_res);

                    const int ntiles_u = face_res.ntilesu(tile_res);
                    const int ntiles_v = face_res.ntilesv(tile_res);
                    const int ntiles = ntiles_u * ntiles_v;

                    face.reset(new TiledFace(face_res, tile_res, _psize));
                    auto *tface = dynamic_cast<TiledFace *>(face.get());

                    for (int i = 0; i < ntiles; i++) {
                        FaceEncoding tface_enc;
                        Ptex::Res tface_res;
                        reader.read(&tface_enc);
                        reader.read(&tface_res);

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

void ExpandedPtex::TiledFace::getPixel(int u, int v, void *result) {
    const int tileu = u >> _tileres.ulog2;
    const int tilev = v >> _tileres.vlog2;
    PtexFaceData *tile = getTile(tilev * _tileres.u() + tileu);
    tile->getPixel(u - (tileu << _tileres.ulog2), v - (tilev << _tileres.vlog2),
                   result);
}
