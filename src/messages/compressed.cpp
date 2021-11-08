#include "compressed.h"

#include <stdexcept>
#include <algorithm>

using namespace std;
using namespace pbrt;

CompressedReader::CompressedReader(const char* buffer, const size_t buffer_len)
    : compressed_data_(buffer),
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
          LZ4F_getFrameInfo(lz4frame_context_, &info, compressed_data_,
                            &src_size);

          compressed_data_ += src_size;
          len_ -= src_size;
          return info;
      }()) {
    lz4frame_options_.stableDst = false;
    memset(lz4frame_options_.reserved, 0, sizeof(lz4frame_options_.reserved));
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
                    &dst_size, compressed_data_, &src_size, &lz4frame_options_);

    compressed_data_ += src_size;
    uncompressed_data_.push(dst_size);
}

uint32_t CompressedReader::next_record_size() {
    if (uncompressed_data_.readable_region().length() < sizeof(uint32_t)) {
        fill_uncompressed_buffer();

        if (uncompressed_data_.readable_region().length() < sizeof(uint32_t))
            throw runtime_error("unexpected end of stream");
    }

    const auto rec_len = *reinterpret_cast<const uint32_t*>(
        uncompressed_data_.readable_region().data());

    return rec_len;
}

void CompressedReader::read(char* dst, size_t len) {
    const size_t rec_len = next_record_size();

    if (rec_len != len) {
        throw runtime_error(string("unexpected size: expected ") +
                            to_string(len) + ", got " + to_string(rec_len));
    }

    uncompressed_data_.pop(sizeof(uint32_t));

    if (not uncompressed_data_.readable_region().empty()) {
        const auto read_len =
            min(uncompressed_data_.readable_region().length(), rec_len);
        memcpy(dst, uncompressed_data_.readable_region().data(), read_len);
        uncompressed_data_.pop(read_len);
        len -= read_len;
        dst += read_len;
    }

    if (not len) return;

    // let's directly decompress to the buffer
    size_t src_size = len_;
    size_t dst_size = len;

    LZ4F_decompress(lz4frame_context_, dst, &dst_size, compressed_data_,
                    &src_size, &lz4frame_options_);

    len -= dst_size;
    len_ -= dst_size;
    compressed_data_ += src_size;

    if (len != 0) {
        throw runtime_error("got " + to_string(len) +
                            " byte(s) fewer than excepted");
    }
}
