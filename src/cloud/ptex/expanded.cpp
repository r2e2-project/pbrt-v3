#include "expanded.h"

#include <iostream>

using namespace std;
using namespace pbrt;

ostream &operator<<(ostream &os, const ExpandedPtex::FaceEncoding &fe) {
    switch (fe) {
    case ExpandedPtex::FaceEncoding::ConstDataPtr:
        os << "ConstDataPtr";
        break;

    case ExpandedPtex::FaceEncoding::Constant:
        os << "Constant";
        break;

    case ExpandedPtex::FaceEncoding::Tiled:
        os << "Tiled";
        break;

    case ExpandedPtex::FaceEncoding::Packed:
        os << "Packed";
        break;

    case ExpandedPtex::FaceEncoding::Error:
        os << "Error";
        break;

    case ExpandedPtex::FaceEncoding::TiledReduced:
        os << "TiledReducedFace";
        break;

    default:
        os << "Unknown";
    }
    return os;
}

ExpandedPtex::FaceEncoding ExpandedPtex::get_face_encoding(
    const PtexFaceData *face_data) {
    if (not face_data) {
        throw runtime_error("face_data == nullptr");
    }

    if (dynamic_cast<const ConstDataPtr *>(face_data)) {
        return FaceEncoding::ConstDataPtr;
    }
    if (dynamic_cast<const ConstantFace *>(face_data)) {
        return FaceEncoding::Constant;
    }
    if (dynamic_cast<const TiledFace *>(face_data)) {
        return FaceEncoding::Tiled;
    }
    if (dynamic_cast<const TiledReducedFace *>(face_data)) {
        return FaceEncoding::TiledReduced;
    }
    if (dynamic_cast<const PackedFace *>(face_data)) {
        return FaceEncoding::Packed;
    }
    if (dynamic_cast<const ErrorFace *>(face_data)) {
        throw runtime_error("encountered error face");
    }

    throw runtime_error("unknown face encoding");
}

pair<void *, int> ExpandedPtex::get_data(PtexFaceData *raw_data,
                                         const int psize) {
    const auto encoding = get_face_encoding(raw_data);

    switch (encoding) {
    case FaceEncoding::ConstDataPtr: {
        auto data = dynamic_cast<ConstDataPtr *>(raw_data);
        return {data->getData(), psize};
    }
    case FaceEncoding::Packed: {
        auto data = dynamic_cast<PackedFace *>(raw_data);
        return {data->getData(), psize * data->res().u() * data->res().v()};
    }
    case FaceEncoding::Tiled:
    case FaceEncoding::TiledReduced: {
        throw runtime_error("call the special function for this encoding");
    }
    }

    throw runtime_error("unknown face encoding");
}

vector<pair<void *, int>> ExpandedPtex::get_tiled_data(PtexFaceData *raw_data,
                                                       const int psize) {
    vector<pair<void *, int>> result;

    auto data = dynamic_cast<TiledFace *>(raw_data);
    // cerr << ", tiles=" << data->ntiles() << " [";
    for (auto i = 0; i < data->ntiles(); i++) {
        auto tile_raw_data = data->getTile(i);
        // cerr << get_face_encoding(tile_raw_data);
        // if (i != data->ntiles() - 1) cerr << ",";
        result.push_back(get_data(tile_raw_data, psize));
    }
    // cerr << "]" << flush;

    return result;
}

vector<pair<void *, int>> ExpandedPtex::get_reduced_tile_data(
    PtexFaceData *raw_data, const int psize) {
    vector<pair<void *, int>> result;

    auto data = dynamic_cast<TiledReducedFace *>(raw_data);
    // cerr << ", tiles=" << data->ntiles() << " [";
    for (auto i = 0; i < data->ntiles(); i++) {
        auto tile_raw_data = data->getTile(i);
        // cerr << get_face_encoding(tile_raw_data);
        // if (i != data->ntiles() - 1) cerr << ",";

        result.push_back(get_data(tile_raw_data, psize));
    }
    // cerr << "]" << flush;

    return result;
}
