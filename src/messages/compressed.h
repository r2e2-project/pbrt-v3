#ifndef PBRT_MESSAGES_COMPRESSED_H
#define PBRT_MESSAGES_COMPRESSED_H

#include <lz4.h>
#include <lz4frame.h>

#include "lite.h"
#include "util/ring_buffer.h"

namespace pbrt {

class CompressedReader : public RecordReader {
  public:
    CompressedReader(const char* buffer, const size_t buffer_len);
    ~CompressedReader();

    void read(char* dst, size_t len) override;
    uint32_t next_record_size() override;

    using RecordReader::read;

  private:
    void fill_uncompressed_buffer();

    const char* buffer_{nullptr};
    size_t len_{0};

    LZ4F_dctx* lz4frame_context_;
    LZ4F_frameInfo_t lz4frame_info_;
    LZ4F_decompressOptions_t lz4frame_options_{};

    RingBuffer uncompressed_data_{16 * 1024 * 1024};  // 16 MiB
};

}  // namespace pbrt

#endif /* PBRT_MESSAGES_COMPRESSED_H */
