#include <Ptexture.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "cloud/manager.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "util/util.h"

using namespace std;

namespace pbrt {

namespace {

class : public PtexInputHandler {
  public:
    Handle open(const char *path) override {
        auto t = global::manager.getInMemoryTexture(path);
        return reinterpret_cast<Handle>(new OpenedTexture(t.first, t.second));
    }

    void seek(Handle handle, int64_t pos) override {
        reinterpret_cast<OpenedTexture *>(handle)->pos = pos;
    }

    size_t read(void *buffer, size_t size, Handle handle) override {
        auto h = reinterpret_cast<OpenedTexture *>(handle);
        const auto ptr = h->data + h->pos;
        const size_t len =
            (h->pos + size > h->length) ? (h->length - h->pos) : size;
        if (len > 0) {
            memcpy(buffer, ptr, len);
            h->pos += len;
        }
        return len;
    }

    bool close(Handle handle) override {
        if (handle) {
            auto t = reinterpret_cast<OpenedTexture *>(handle);
            delete t;
            return true;
        }

        return false;
    }

    const char *lastError() override { return nullptr; }

  private:
    struct OpenedTexture {
        OpenedTexture(const char *data, const size_t length)
            : data(data), length(length) {}

        const char *data;
        size_t length;
        int64_t pos{0};
    };

} in_memory_input_handler;

}  // anonymous namespace

size_t load_all_textures(const string &treelet_path) {
    vector<char> treelet_buffer;
    size_t texture_count = 0;

    unique_ptr<char[]> texture_buffer{nullptr};

    ifstream fin{treelet_path, ios::binary | ios::ate};

    if (!fin.good()) {
        throw runtime_error("Could not open treelet file: " + treelet_path);
    }

    streamsize size = fin.tellg();
    fin.seekg(0, ios::beg);

    treelet_buffer.resize(size);
    fin.read(treelet_buffer.data(), size);

    unique_ptr<RecordReader> reader;
    if (*reinterpret_cast<const uint32_t *>(treelet_buffer.data()) ==
        0x184D2204) {
        reader = make_unique<CompressedReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    } else {
        reader = make_unique<LiteRecordReader>(treelet_buffer.data(),
                                               treelet_buffer.size());
    }

    reader->skip(reader->read<uint32_t>());

    const uint32_t included_texture_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_texture_count; i++) {
        reader->skip(1);  // skipping the id

        const auto l = reader->next_record_size();
        texture_buffer = make_unique<char[]>(l);
        reader->read(texture_buffer.get(), l);

        pbrt::global::manager.addInMemoryTexture(
            "TEX" + to_string(texture_count), move(texture_buffer), l);

        texture_count++;
    }

    return texture_count;
}

}  // namespace pbrt

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " TREELET-FILE" << endl;
}

void print_cache_stats(Ptex::PtexCache *cache) {
    Ptex::PtexCache::Stats stats;
    cache->getStats(stats);
    cout << "[PTEX] memUsed=" << stats.memUsed
         << ",peakMemUsed=" << stats.peakMemUsed << endl;
}

uint64_t get_current_rss() {
    static const int pagesize = getpagesize();

    ifstream fin{"/proc/self/statm"};

    uint64_t size, resident, shared, text, lib, data, dt;
    fin >> size >> resident >> shared >> text >> lib >> data >> dt;

    return resident * pagesize;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        usage((argc <= 0) ? "ptextool" : argv[0]);
        return EXIT_FAILURE;
    }

    const string treelet_path{argv[1]};

    cerr << "Loading all textures in the treelet... ";
    const auto texture_count = pbrt::load_all_textures(treelet_path);
    cerr << "done. (" << texture_count << " texture"
         << (texture_count == 1 ? "" : "s") << " loaded)" << endl;

    const int max_files = 1000;
    const size_t max_mem = 0;
    const bool premultiply = true;

    PtexCache *cache =
        Ptex::PtexCache::create(max_files, max_mem, premultiply,
                                &pbrt::in_memory_input_handler, nullptr);

    Ptex::String error;
    Ptex::PtexTexture *texture = nullptr;

    auto start = chrono::steady_clock::now();

    for (size_t texture_id = 0; texture_id < texture_count; texture_id++) {
        const string texture_name = "TEX" + to_string(texture_id);
        texture = cache->get(texture_name.c_str(), error);

        for (int i = 0; i < texture->numFaces(); i++) {
            float result[3];
            auto face_data = texture->getData(i);
            auto res_x = face_data->res().ulog2;
            auto res_y = face_data->res().vlog2;

            while (res_x >= 0 && res_y >= 0) {
                Ptex::Res new_res{res_x, res_y};
                face_data = texture->getData(i, new_res);

                for (int x = 0; x < new_res.u(); x++) {
                    for (int y = 0; y < new_res.v(); y++) {
                        face_data->getPixel(x, y, result);
                    }
                }

                res_x--;
                res_y--;
            }

            if (chrono::steady_clock::now() - start >= chrono::seconds{1}) {
                start = chrono::steady_clock::now();
                const auto rss = get_current_rss();
                cerr << "\33[2K\r"
                     << "Processed " << texture_id << "/" << texture_count
                     << ", RSS = " << pbrt::format_bytes(rss);
            }
        }
    }

    cerr << endl;

    return EXIT_SUCCESS;
}
