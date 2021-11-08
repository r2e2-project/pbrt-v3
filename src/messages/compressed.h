#ifndef PBRT_MESSAGES_COMPRESSED_H
#define PBRT_MESSAGES_COMPRESSED_H

#include <lz4.h>
#include <lz4frame.h>

#include "util/ring_buffer.h"

namespace pbrt {

class CompressedReader {
  public:
    CompressedReader(const char* buffer, const size_t buffer_len);
    ~CompressedReader();

    template <class T>
    T read();

    uint32_t next_record_size();

    //! reads exactly `dst_len` bytes, throws an exception if failed
    void read(char* dst, size_t len);

  private:
    void fill_uncompressed_buffer();

    const char* compressed_data_;
    size_t len_;

    LZ4F_dctx* lz4frame_context_;
    LZ4F_frameInfo_t lz4frame_info_;
    LZ4F_decompressOptions_t lz4frame_options_{};

    RingBuffer uncompressed_data_{16 * 1024 * 1024};  // 16 MiB
};

template <class T>
T CompressedReader::read() {
    T res;
    read(&res, sizeof(T));
    return res;
}

}  // namespace pbrt

#endif /* PBRT_MESSAGES_COMPRESSED_H */
