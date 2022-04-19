#include <Ptexture.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "cloud/manager.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "util/util.h"

using namespace std;
using namespace chrono;

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

void load_all_textures(const string &treelet_path,
                       vector<size_t> &texture_sizes) {
    vector<char> treelet_buffer;

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

        pbrt::global::manager.addInMemoryTexture("TEX" + to_string(i),
                                                 move(texture_buffer), l);

        texture_sizes.push_back(l);
    }
}

}  // namespace pbrt

void usage(const char *argv0) {
    cerr << "Usage: " << argv0 << " TREELET-FILE TIME THREADS" << endl;
}

uint64_t get_ptex_mem_used(Ptex::PtexCache *cache) {
    Ptex::PtexCache::Stats stats;
    cache->getStats(stats);
    return stats.memUsed;
}

uint64_t get_current_rss() {
    static const int pagesize = getpagesize();

    ifstream fin{"/proc/self/statm"};

    uint64_t size, resident, shared, text, lib, data, dt;
    fin >> size >> resident >> shared >> text >> lib >> data >> dt;

    return resident * pagesize;
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        usage((argc <= 0) ? "ptextool" : argv[0]);
        return EXIT_FAILURE;
    }

    const string treelet_path{argv[1]};
    const seconds total_runtime{stoul(argv[2])};
    const size_t thread_count{stoul(argv[3])};

    vector<size_t> texture_sizes;

    cerr << "Loading all textures in the treelet... ";
    pbrt::load_all_textures(treelet_path, texture_sizes);
    const auto texture_count = texture_sizes.size();
    cerr << "done. (" << texture_count << " texture"
         << (texture_count == 1 ? "" : "s") << " loaded)" << endl;

    if (texture_count == 0) {
        cerr << "Not textures found, exiting." << endl;
        return EXIT_FAILURE;
    }

    const int max_files = 1000;
    const size_t max_mem = 1ull << 30;  // 1 GB
    const bool premultiply = true;

    Ptex::PtexPtr<Ptex::PtexCache> cache{
        Ptex::PtexCache::create(max_files, max_mem, premultiply,
                                &pbrt::in_memory_input_handler, nullptr)};

    const auto start_time = steady_clock::now();
    atomic<size_t> sampled_count{0};

    vector<thread> threads;

    for (size_t thread_id = 0; thread_id < thread_count; thread_id++) {
        threads.emplace_back([&start_time, &total_runtime, &cache,
                              &texture_sizes, &sampled_count] {
            Ptex::String error;
            Ptex::PtexPtr<Ptex::PtexTexture> texture{nullptr};

            mt19937 gen;
            gen.seed(1000);

            // randomly select textures relative to their size
            discrete_distribution<size_t> texture_id_gen{texture_sizes.begin(),
                                                         texture_sizes.end()};

            for (auto now = steady_clock::now();
                 now - start_time <= total_runtime; now = steady_clock::now()) {
                const auto texname =
                    "TEX" + to_string(0);  // texture_id_gen(gen));
                texture.reset();
                texture.reset(cache->get(texname.c_str(), error));

                uniform_int_distribution<int> face_id_dist{
                    0, texture->numFaces() - 1};
                const int i = face_id_dist(gen);

                float result[3];

                // randomly select a resultion
                Ptex::PtexPtr<Ptex::PtexFaceData> face_data{
                    texture->getData(i)};
                uniform_int_distribution<int8_t> ures_dist{
                    0, face_data->res().ulog2};
                uniform_int_distribution<int8_t> vres_dist{
                    0, face_data->res().vlog2};
                Ptex::Res res{ures_dist(gen), vres_dist(gen)};

                // randomly select a pixel
                uniform_int_distribution<int> x_dist{0, res.u() - 1};
                uniform_int_distribution<int> y_dist{0, res.v() - 1};
                face_data.reset(texture->getData(i, res));
                face_data->getPixel(x_dist(gen), y_dist(gen), result);
                sampled_count++;
            }
        });
    }

    while (true) {
        const auto now = steady_clock::now();
        const auto next_wake_up = now + seconds{1};
        const auto rss = get_current_rss();

        cerr << "\33[2K\r"
             << "Processed " << pbrt::format_num(sampled_count) << " samples"
             << ", RSS = " << pbrt::format_bytes(rss)
             << ", Ptex = " << pbrt::format_bytes(get_ptex_mem_used(cache));

        if (now - start_time > total_runtime) {
            break;
        }

        this_thread::sleep_until(next_wake_up);
    }

    for (auto &thread : threads) {
        thread.join();
    }

    cerr << endl;

    return EXIT_SUCCESS;
}
