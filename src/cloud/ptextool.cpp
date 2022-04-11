#include <Ptexture.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "cloud/manager.h"
#include "messages/compressed.h"
#include "messages/lite.h"

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

pair<unique_ptr<char[]>, size_t> get_texture_data(const string &treelet_path,
                                                  const uint32_t texture_id) {
    vector<char> treelet_buffer;
    unique_ptr<char[]> result{nullptr};

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

    size_t total_texture_size = 0;
    const uint32_t included_texture_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_texture_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();

        if (id == texture_id) {
            result = make_unique<char[]>(l);
            reader->read(result.get(), l);
            return make_pair(move(result), l);
        } else {
            reader->skip(1);
        }
    }

    throw runtime_error("Texture not found: " + to_string(texture_id));
}

}  // namespace pbrt

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " TREELET-FILE TEXTURE-ID" << endl;
}

void print_cache_stats(Ptex::PtexCache *cache) {
    Ptex::PtexCache::Stats stats;
    cache->getStats(stats);
    cout << "[PTEX] memUsed=" << stats.memUsed
         << ",peakMemUsed=" << stats.peakMemUsed << endl;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        usage((argc <= 0) ? "ptextool" : argv[0]);
        return EXIT_FAILURE;
    }

    const string treelet_path{argv[1]};
    const uint32_t texture_id = static_cast<uint32_t>(stoul(argv[2]));

    {
        auto raw_texture = pbrt::get_texture_data(treelet_path, texture_id);
        pbrt::global::manager.addInMemoryTexture(
            "TEX0", move(raw_texture.first), raw_texture.second);

        cerr << "Texture " << texture_id << " loaded (" << raw_texture.second
             << " bytes)." << endl;
    }

    const int max_files = 1000;
    const size_t max_mem = 0;
    const bool premultiply = true;

    PtexCache *cache =
        Ptex::PtexCache::create(max_files, max_mem, premultiply,
                                &pbrt::in_memory_input_handler, nullptr);

    Ptex::String error;
    Ptex::PtexTexture *texture = cache->get("TEX0", error);

    auto start = chrono::steady_clock::now();

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

        if (chrono::steady_clock::now() - start > chrono::seconds{1}) {
            start = chrono::steady_clock::now();

            print_cache_stats(cache);
            cache->purge(texture);
            texture->release();
            texture = cache->get("TEX0", error);
            print_cache_stats(cache);
        }
    }

    return EXIT_SUCCESS;
}
