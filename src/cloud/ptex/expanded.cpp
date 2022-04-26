#include "expanded.h"

#include <iostream>

using namespace std;
using namespace pbrt;

ostream &operator<<(ostream &os, const ptex::util::FaceEncoding &fe) {
    using namespace ptex::util;

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

ptex::util::FaceEncoding ptex::util::get_face_encoding(
    const PtexFaceData *face_data) {
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

pair<void *, int> ptex::util::get_data(PtexFaceData *raw_data,
                                       const int psize) {
    const auto encoding = get_face_encoding(raw_data);

    switch (encoding) {
    case FaceEncoding::ConstDataPtr: {
        auto data = dynamic_cast<Ptex::PtexReader::ConstDataPtr *>(raw_data);
        return {data->getData(), psize};
    }
    case FaceEncoding::Packed: {
        auto data = dynamic_cast<Ptex::PtexReader::PackedFace *>(raw_data);
        return {data->getData(), psize * data->res().u() * data->res().v()};
    }
    case FaceEncoding::Tiled:
    case FaceEncoding::TiledReduced: {
        throw runtime_error("call the special function for this encoding");
    }
    }

    throw runtime_error("unknown face encoding");
}
