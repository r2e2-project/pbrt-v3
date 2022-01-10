#include <iostream>
#include <string>

#include "cloud/manager.h"
#include "include/pbrt/common.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "messages/utils.h"
#include "util/exception.h"
#include "util/path.h"

using namespace std;
using namespace pbrt;

auto& _manager = global::manager;

void usage(const char* argv0) { cerr << argv0 << " SCENE-PATH" << endl; }

void print_deps(const protobuf::Manifest& manifest, const ObjectKey& root,
                const int level) {
    for (const auto& obj_proto : manifest.objects()) {
        auto obj = from_protobuf(obj_proto.id());
        if (obj == root) {
            for (const auto& dep_proto : obj_proto.dependencies()) {
                auto dep = from_protobuf(dep_proto);
                cout << string(level * 2, ' ') << "\u21b3 "
                     << _manager.getFileName(dep.type, dep.id)
                     << "  \033[38;5;242m"
                     << format_bytes(roost::file_size(
                            _manager.getFilePath(dep.type, dep.id)))
                     << "\033[0m" << endl;
                print_deps(manifest, dep, level + 1);
            }
        }
    }
}

void print_treelet_info(const uint32_t treelet_id) {
    vector<char> treelet_buffer;

    const string treelet_path =
        _manager.getScenePath() + "/" +
        _manager.getFileName(ObjectType::Treelet, treelet_id);

    ifstream fin{treelet_path, ios::binary | ios::ate};

    if (!fin.good()) {
        throw runtime_error("Could not open treelet file: " + treelet_path);
    }

    streamsize size = fin.tellg();
    fin.seekg(0, ios::beg);

    treelet_buffer.resize(size);
    fin.read(treelet_buffer.data(), size);

    unique_ptr<RecordReader> reader;
    if (*reinterpret_cast<const uint32_t*>(treelet_buffer.data()) ==
        0x184D2204) {
        reader = make_unique<CompressedReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    } else {
        reader = make_unique<LiteRecordReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    }

    map<string, size_t> tex_sizes;
    map<uint32_t, string> stex_texs, ftex_texs;

    size_t total_image_partition_size = 0;
    const uint32_t included_image_partitions = reader->read<uint32_t>();
    for (size_t i = 0; i < included_image_partitions; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();
        reader->read<string>();
        total_image_partition_size += l;
    }

    size_t total_texture_size = 0;
    const uint32_t included_texture_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_texture_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();
        reader->read<string>();
        total_texture_size += l;
        tex_sizes[_manager.getFileName(ObjectType::Texture, id)] = l;

        cerr << "T" << treelet_id << " TEX" << id << " " << l << endl;
    }

    size_t total_spectrum_size = 0;
    const uint32_t included_spectrum_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_spectrum_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto data = reader->read<string>();
        total_spectrum_size += data.length();

        protobuf::SpectrumTexture stex_proto;
        stex_proto.ParseFromString(data);
        auto& params = stex_proto.params();
        for (size_t j = 0; j < params.strings_size(); j++) {
            if (params.strings(j).name() == "filename") {
                stex_texs[id] = params.strings(j).values(0);
                break;
            }
        }
    }

    size_t total_float_size = 0;
    const uint32_t included_float_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_float_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto data = reader->read<string>();
        total_float_size += data.length();

        protobuf::FloatTexture ftex_proto;
        ftex_proto.ParseFromString(data);
        auto& params = ftex_proto.params();
        for (size_t j = 0; j < params.strings_size(); j++) {
            if (params.strings(j).name() == "filename") {
                ftex_texs[id] = params.strings(j).values(0);
                break;
            }
        }
    }

    size_t total_material_size = 0;
    const uint32_t included_material_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_material_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto data = reader->read<string>();
        total_material_size += data.length();

        protobuf::Material mat_proto;
        mat_proto.ParseFromString(data);

        vector<string> texture_key;

        for (auto& tex : mat_proto.spectrum_textures()) {
            if (stex_texs.count(tex.second)) {
                texture_key.push_back(stex_texs[tex.second]);
            }
        }

        for (auto& tex : mat_proto.float_textures()) {
            if (ftex_texs.count(tex.second)) {
                texture_key.push_back(ftex_texs[tex.second]);
            }
        }

        sort(texture_key.begin(), texture_key.end());

        cerr << "T" << treelet_id << " MAT" << id << " " << data.length();
        for (const auto& t : texture_key) cerr << " " << t;
        cerr << endl;
    }

    const uint32_t included_mesh_count = reader->read<uint32_t>();

    cout << "\u21b3 PART: " << included_image_partitions << "  \033[38;5;242m"
         << format_bytes(total_image_partition_size) << "\033[0m" << endl
         << "\u21b3 TEX:  " << included_texture_count << "  \033[38;5;242m"
         << format_bytes(total_texture_size) << "\033[0m" << endl
         << "\u21b3 STEX: " << included_spectrum_count << "  \033[38;5;242m"
         << format_bytes(total_spectrum_size) << "\033[0m" << endl
         << "\u21b3 FTEX: " << included_float_count << "  \033[38;5;242m"
         << format_bytes(total_float_size) << "\033[0m" << endl
         << "\u21b3 MAT:  " << included_material_count << "  \033[38;5;242m"
         << format_bytes(total_material_size) << "\033[0m" << endl
         << "\u21b3 MESH: " << included_mesh_count << endl;
}

int main(int argc, char* argv[]) {
    try {
        if (argc <= 0) {
            abort();
        }

        if (argc != 2) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        const string scene_path{argv[1]};
        _manager.init(scene_path);

        // read the manifest file
        protobuf::Manifest manifest_proto;
        _manager.GetReader(ObjectType::Manifest)->read(&manifest_proto);

        for (size_t i = 0; i < _manager.treeletCount(); i++) {
            cout << "T" << i << "  ("
                 << format_bytes(roost::file_size(
                        _manager.getFilePath(ObjectType::Treelet, i)))
                 << ")" << endl;
            print_treelet_info(i);
            print_deps(manifest_proto, {ObjectType::Treelet, i}, 0);
        }
    } catch (const exception& e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
