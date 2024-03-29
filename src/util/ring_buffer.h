#pragma once

#include <vector>

#include "file_descriptor.h"
#include "simple_string_span.h"

namespace pbrt {

class MMap_Region {
    char* addr_;
    size_t length_;

  public:
    MMap_Region(char* const addr, const size_t length, const int prot,
                const int flags, const int fd, const off_t offset = 0);

    ~MMap_Region();

    MMap_Region(MMap_Region&& other)
        : addr_(other.addr_), length_(other.length_) {
        other.addr_ = nullptr;
        other.length_ = 0;
    }

    MMap_Region& operator=(MMap_Region&& other) {
        addr_ = other.addr_;
        length_ = other.length_;

        other.addr_ = nullptr;
        other.length_ = 0;

        return *this;
    }

    /* Disallow copying */
    MMap_Region(const MMap_Region& other) = delete;
    MMap_Region& operator=(const MMap_Region& other) = delete;

    char* addr() const { return addr_; }
    size_t length() const { return length_; }
};

class RingBuffer {
    size_t next_index_to_write_ = 0;
    size_t bytes_stored_ = 0;

    FileDescriptor fd_;
    MMap_Region virtual_address_space_, first_mapping_, second_mapping_;

  public:
    explicit RingBuffer(const size_t capacity);

    size_t capacity() const { return first_mapping_.length(); }

    simple_string_span writable_region();
    string_view writable_region() const;
    void push(const size_t num_bytes);

    string_view readable_region() const;
    void pop(const size_t num_bytes);

    size_t write(const string_view str);
};

}  // namespace pbrt
