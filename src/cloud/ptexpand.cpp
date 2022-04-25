
#include <iostream>

//

#include <PtexReader.h>
#include <Ptexture.h>

using namespace std;

namespace pbrt {

class ExpandedPtex : public Ptex::PtexReader {
  public:
    enum class FaceEncoding {
        ConstDataPtr,
        Constant,
        Tiled,
        Packed,
        TiledReducedFace,
        Error,
    };

    static FaceEncoding get_face_encoding(const PtexFaceData *face_data) {
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
            return FaceEncoding::TiledReducedFace;
        }
        if (dynamic_cast<const PackedFace *>(face_data)) {
            return FaceEncoding::Packed;
        }
        if (dynamic_cast<const ErrorFace *>(face_data)) {
            throw runtime_error("encountered error face");
        }

        throw runtime_error("unknown face encoding");
    }
};

}  // namespace pbrt

using namespace pbrt;

std::ostream &operator<<(std::ostream &os,
                         const ExpandedPtex::FaceEncoding &fe) {
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

    case ExpandedPtex::FaceEncoding::TiledReducedFace:
        os << "TiledReducedFace";
        break;

    default:
        os << "Unknown";
    }
    return os;
}

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " INPUT" << endl;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        usage((argc <= 0) ? "ptextract" : argv[0]);
        return EXIT_FAILURE;
    }

    const string ptex_file{argv[1]};

    Ptex::String error;
    PtexPtr<Ptex::PtexReader> ptex_texture{
        new Ptex::PtexReader(false, nullptr, nullptr)};
    if (!ptex_texture->open(ptex_file.c_str(), error)) {
        throw runtime_error("cannot open the texture: " + ptex_file +
                            " (reason: " + error + ")");
    }

    const auto num_faces = ptex_texture->numFaces();

    cerr << "* Texture info: " << endl << "  - Faces = " << num_faces << endl;

    for (int i = 0; i < num_faces; i++) {
        auto &face_info = ptex_texture->getFaceInfo(i);

        cerr << "[F" << i << "]" << endl;

        if (face_info.res.ulog2 < 0 or face_info.res.vlog2 < 0) {
            throw runtime_error("unsupported face resolution");
        }

        for (int8_t res_u = 0; res_u <= face_info.res.ulog2; res_u++) {
            for (int8_t res_v = 0; res_v <= face_info.res.vlog2; res_v++) {
                Ptex::Res new_res{res_u, res_v};
                PtexFaceData *data = ptex_texture->getData(i, new_res);

                cerr << "  (" << static_cast<int>(res_u) << "x"
                     << static_cast<int>(res_v)
                     << ") enc=" << ExpandedPtex::get_face_encoding(data)
                     << endl;
            }
        }
    }

    return EXIT_SUCCESS;
}
