#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>

#include "cloud/manager.h"
#include "include/pbrt/common.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "messages/utils.h"
#include "parallel.h"
#include "util/exception.h"
#include "util/path.h"

using namespace std;
using namespace pbrt;

auto &_manager = global::manager;

void usage(const char *argv0) { cerr << argv0 << " SCENE-PATH" << endl; }

bool is_material_treelet(const uint32_t treelet_id) {
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

    const uint32_t included_image_partitions = reader->read<uint32_t>();

    if (included_image_partitions) {
        return true;
    }

    const uint32_t included_texture_count = reader->read<uint32_t>();
    if (included_texture_count) {
        return true;
    }

    const uint32_t included_spectrum_count = reader->read<uint32_t>();
    if (included_spectrum_count) {
        return true;
    }

    const uint32_t included_float_count = reader->read<uint32_t>();
    if (included_float_count) {
        return true;
    }

    const uint32_t included_material_count = reader->read<uint32_t>();
    if (included_material_count) {
        return true;
    }

    return false;
}

int main(int argc, char const *argv[]) {
    try {
        if (argc <= 0) {
            abort();
        }

        if (argc != 2) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        FLAGS_logtostderr = false;
        FLAGS_minloglevel = 3;

        const roost::path scene_path{argv[1]};
        _manager.init(scene_path.string());

        atomic<size_t> total_material_size{0};
        atomic<size_t> total_geometry_size{0};

        ParallelInit();

        ParallelFor(
            [&](const uint64_t i) {
                if (is_material_treelet(i)) {
                    total_material_size += roost::file_size(
                        _manager.getFilePath(ObjectType::Treelet, i));
                } else {
                    total_geometry_size += roost::file_size(
                        _manager.getFilePath(ObjectType::Treelet, i));
                }
            },
            _manager.treeletCount());

        cout << "total_material_size = " << total_material_size << " bytes"
             << endl
             << "total_geometry_size = " << total_geometry_size << " bytes"
             << endl;

        ParallelCleanup();
    } catch (const exception &e) {
        print_exception(argv[0], e);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
