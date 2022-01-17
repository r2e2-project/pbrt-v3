#include <fstream>
#include <iostream>
#include <memory>

#include "cloud/manager.h"
#include "include/pbrt/common.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "messages/utils.h"
#include "util/exception.h"
#include "util/path.h"

using namespace std;
using namespace pbrt;

auto &_manager = global::manager;

void usage(const char *argv0) {
    cerr << argv0 << " SCENE-PATH TREELET-NUM" << endl;
}

void print_treelet_info(const uint32_t treelet_id) {
    const string treelet_path =
        _manager.getScenePath() + "/" +
        _manager.getFileName(ObjectType::Treelet, treelet_id);

    string treelet_buffer = roost::read_file(treelet_path);

    unique_ptr<RecordReader> reader;
    if (*reinterpret_cast<const uint32_t *>(treelet_buffer.data()) ==
        0x184D2204) {
        reader = make_unique<CompressedReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    } else {
        reader = make_unique<LiteRecordReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    }

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
    }

    size_t total_spectrum_size = 0;
    const uint32_t included_spectrum_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_spectrum_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();
        reader->read<string>();
        total_spectrum_size += l;
    }

    size_t total_float_size = 0;
    const uint32_t included_float_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_float_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();
        reader->read<string>();
        total_float_size += l;
    }

    size_t total_material_size = 0;
    const uint32_t included_material_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_material_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();
        reader->read<string>();
        total_material_size += l;
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

int main(int argc, char const *argv[]) {
    try {
        if (argc <= 0) {
            abort();
        }

        if (argc != 3) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        FLAGS_logtostderr = false;
        FLAGS_minloglevel = 3;
        PbrtOptions.nThreads = 1;

        const roost::path scene_path{argv[1]};
        const uint32_t treelet_id = stoul(argv[2]);
        _manager.init(scene_path.string());

        print_treelet_info(treelet_id);
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
