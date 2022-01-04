#include "lite.h"

#include <cstring>
#include <iostream>

using namespace std;

template <class T>
T RecordReader::read() {
    T res;
    read(reinterpret_cast<char*>(&res), sizeof(T));
    return res;
}

template <>
std::string RecordReader::read() {
    std::string res;
    res.resize(next_record_size());
    read(reinterpret_cast<char*>(&res[0]), res.size());
    return res;
}

template uint32_t RecordReader::read<uint32_t>();
template uint64_t RecordReader::read<uint64_t>();

LiteRecordReader::LiteRecordReader(const char* buffer, const size_t len)
    : buffer_(buffer), len_(len), end_(buffer_ + len_) {}

uint32_t LiteRecordReader::next_record_size() {
    if (buffer_ + sizeof(uint32_t) > end_) {
        throw runtime_error("unexcepted end of stream");
    }

    return *reinterpret_cast<const uint32_t*>(buffer_);
}

void LiteRecordReader::read(char* dst, size_t len) {
    const auto rec_len = next_record_size();

    if (rec_len != len) {
        throw runtime_error(string("unexpected size: expected ") +
                            to_string(len) + ", got " + to_string(rec_len));
    }

    buffer_ += sizeof(uint32_t);

    if (dst != nullptr) {
        memcpy(dst, buffer_, rec_len);
    }

    buffer_ += rec_len;
}

void LiteRecordReader::skip(const size_t n) {
    if (n == 0) return;
    for (size_t i = 0; i < n; i++) {
        read(nullptr, next_record_size());
    }
}

FileRecordReader::FileRecordReader(const string& path)
    : fin_(path, ios::binary) {}

uint32_t FileRecordReader::next_record_size() {
    if (next_size_ > 0) return next_size_;

    if (fin_.good() && !fin_.eof()) {
        fin_.read(reinterpret_cast<char*>(&next_size_), sizeof(uint32_t));
        if (fin_.gcount() == sizeof(uint32_t)) return next_size_;
    }

    throw runtime_error("unexcepted end of stream");
}

void FileRecordReader::read(char* dst, size_t len) {
    const auto rec_len = next_record_size();

    if (rec_len != len) {
        throw runtime_error(string("unexpected size: expected ") +
                            to_string(len) + ", got " + to_string(rec_len));
    }

    if (dst) {
        fin_.read(dst, len);
    } else {
        fin_.ignore(len);
    }

    next_size_ = 0;
}

void FileRecordReader::skip(const size_t n) {
    if (n == 0) return;
    for (size_t i = 0; i < n; i++) {
        read(nullptr, next_record_size());
    }
}

void LiteRecordWriter::write(const char* buf, const uint32_t len) {
    fout_.write(reinterpret_cast<const char*>(&len), sizeof(len));
    fout_.write(buf, len);
}

void LiteRecordWriter::write_raw(const string& str) {
    fout_.write(str.data(), str.length());
}
