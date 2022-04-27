#include "expanded.h"

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

tuple<FaceEncoding, void *, int> ptex::util::get_data(PtexFaceData *raw_data,
                                                      const int psize) {
    const auto encoding = get_face_encoding(raw_data);

    switch (encoding) {
    case FaceEncoding::ConstDataPtr: {
        auto data = dynamic_cast<Ptex::PtexReader::ConstDataPtr *>(raw_data);
        return {encoding, data->getData(), psize};
    }
    case FaceEncoding::Packed: {
        auto data = dynamic_cast<Ptex::PtexReader::PackedFace *>(raw_data);
        return {encoding, data->getData(),
                psize * data->res().u() * data->res().v()};
    }
    case FaceEncoding::Tiled:
    case FaceEncoding::TiledReduced: {
        throw runtime_error("call the special function for this encoding");
    }
    }

    throw runtime_error("unknown face encoding");
}

ExpandedPtex::ExpandedPtex(const string &path, const char *data,
                           const size_t data_len)
    : _path(path) {
    LiteRecordReader reader{data, data_len};
    reader.read(&_i);

    _faceinfo.resize(_i.numFaces);
    reader.read(reinterpret_cast<char *>(_faceinfo.data()),
                _i.numFaces * sizeof(_faceinfo[0]));

    for (int i = 0; i < _i.numFaces; i++) {
        auto &fi = _faceinfo[i];

        Ptex::Res max_res;
        FaceEncoding face_encoding;

        do {
            reader.read(&max_res);
            reader.read(&face_encoding);

            switch (face_encoding) {
            case FaceEncoding::ConstDataPtr:
            case FaceEncoding::Constant:
            case FaceEncoding::Packed:
            case FaceEncoding::Tiled:
            case FaceEncoding::TiledReduced: {
                int ntilesu, ntilesv;
                reader.read(&ntilesu);
                reader.read(&ntilesv);
                // we need to read the tiles...
            }

            case FaceEncoding::Error:
            default:
                throw runtime_error("unkown face encoding");
            }

        } while (max_res.ulog2 > 0 or max_res.vlog2 > 0);
    }
}

void ExpandedPtex::TiledFace::getPixel(int u, int v, void *result) {
    const int tileu = u >> _tileres.ulog2;
    const int tilev = v >> _tileres.vlog2;
    PtexFaceData *tile = getTile(tilev * _tileres.u() + tileu);
    tile->getPixel(u - (tileu << _tileres.ulog2), v - (tilev << _tileres.vlog2),
                   result);
}
