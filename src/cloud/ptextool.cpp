#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "messages/compressed.h"
#include "messages/lite.h"

using namespace std;
using namespace pbrt;

void usage(const char* argv0) {
    cerr << "Usage: " << argv0 << " TREELET-FILE TEXTURE-ID" << endl;
}

string get_texture_data(const string& treelet_path, const uint32_t texture_id) {
    vector<char> treelet_buffer;

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

    reader->skip(reader->read<uint32_t>());

    size_t total_texture_size = 0;
    const uint32_t included_texture_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_texture_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const auto l = reader->next_record_size();

        if (id == texture_id) {
            return reader->read<string>();
        } else {
            reader->skip(1);
        }
    }

    throw runtime_error("Texture not found: " + to_string(texture_id));
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        usage((argc <= 0) ? "ptextool" : argv[0]);
        return EXIT_FAILURE;
    }

    const string treelet_path{argv[1]};
    const uint32_t texture_id = static_cast<uint32_t>(stoul(argv[2]));
    const string raw_texture = get_texture_data(treelet_path, texture_id);

    cout << raw_texture.length() << endl;

    return EXIT_SUCCESS;
}
