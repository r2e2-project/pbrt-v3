#include "compressed.h"

#include <algorithm>
#include <stdexcept>

using namespace std;
using namespace pbrt;

CompressedReader::CompressedReader(const char* buffer, const size_t buffer_len)
    : buffer_(buffer),
      len_(buffer_len),
      lz4frame_context_([] {
          LZ4F_dctx* tmp;
          if (LZ4F_isError(
                  LZ4F_createDecompressionContext(&tmp, LZ4F_getVersion()))) {
              throw runtime_error("could not create lz4frame context");
          }
          return tmp;
      }()),
      lz4frame_info_([this] {
          LZ4F_frameInfo_t info;
          size_t src_size = len_;
          LZ4F_getFrameInfo(lz4frame_context_, &info, buffer_, &src_size);

          buffer_ += src_size;
          len_ -= src_size;
          return info;
      }()) {
    lz4frame_options_.stableDst = false;
    // memset(lz4frame_options_.reserved, 0, sizeof(lz4frame_options_.reserved));
}

CompressedReader::~CompressedReader() {
    LZ4F_freeDecompressionContext(lz4frame_context_);
}

void CompressedReader::fill_uncompressed_buffer() {
    size_t dst_size = uncompressed_data_.writable_region().length();
    size_t src_size = len_;

    if (dst_size == 0) {
        return;
    }

    LZ4F_decompress(lz4frame_context_,
                    uncompressed_data_.writable_region().mutable_data(),
                    &dst_size, buffer_, &src_size, &lz4frame_options_);

    buffer_ += src_size;
    uncompressed_data_.push(dst_size);
}

uint32_t CompressedReader::next_record_size() {
    if (uncompressed_data_.readable_region().length() < sizeof(uint32_t)) {
        fill_uncompressed_buffer();

        if (uncompressed_data_.readable_region().length() < sizeof(uint32_t))
            throw runtime_error("unexpected end of stream");
    }

    return *reinterpret_cast<const uint32_t*>(
        uncompressed_data_.readable_region().data());
}

void CompressedReader::read(char* dst, size_t dst_len) {
    const size_t rec_len = next_record_size();

    if (rec_len != dst_len) {
        throw runtime_error(string("unexpected size: expected ") +
                            to_string(dst_len) + ", got " + to_string(rec_len));
    }

    uncompressed_data_.pop(sizeof(uint32_t));

    if (not uncompressed_data_.readable_region().empty()) {
        const auto read_len =
            min(uncompressed_data_.readable_region().length(), rec_len);
        memcpy(dst, uncompressed_data_.readable_region().data(), read_len);
        uncompressed_data_.pop(read_len);
        dst_len -= read_len;
        dst += read_len;
    }

    if (not dst_len) return;

    // let's directly decompress to the buffer
    size_t src_size = len_;
    size_t dst_size = dst_len;

    const auto decomp_result =
        LZ4F_decompress(lz4frame_context_, dst, &dst_size, buffer_, &src_size,
                        &lz4frame_options_);

    if (LZ4F_isError(decomp_result)) {
        throw runtime_error("decompression failed: "s +
                            LZ4F_getErrorName(decomp_result));
    }

    dst_len -= dst_size;
    len_ -= src_size;
    buffer_ += src_size;

    if (dst_len != 0) {
        throw runtime_error("got " + to_string(dst_len) +
                            " byte(s) fewer than excepted (record length: " +
                            to_string(rec_len) + ")");
    }
}

unique_ptr<RecordReader> RecordReader::get(const char* buffer,
                                           const size_t buffer_len) {
    if (buffer_len >= sizeof(uint32_t) &&
        *reinterpret_cast<const uint32_t*>(buffer) == 0x184D2204) {
        return make_unique<CompressedReader>(buffer, buffer_len);
    }

    return make_unique<LiteRecordReader>(buffer, buffer_len);
}
