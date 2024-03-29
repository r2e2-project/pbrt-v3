#ifndef PBRT_MESSAGES_LITE_H
#define PBRT_MESSAGES_LITE_H

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

class RecordReader {
  public:
    //! reads exactly `dst_len` bytes, throws an exception if failed
    virtual void read(char* dst, size_t len) = 0;
    virtual uint32_t next_record_size() = 0;
    virtual void skip(const size_t n) { throw std::runtime_error("not impl"); }

    template <class T>
    T read();

    template <class T>
    void read(T* t) {
        read(reinterpret_cast<char*>(t), sizeof(T));
    }

    static std::unique_ptr<RecordReader> get(const char* buffer,
                                             const size_t buffer_len);
};

class LiteRecordReader : public RecordReader {
  public:
    LiteRecordReader(const char* buffer, const size_t len);

    uint32_t next_record_size() override;
    void read(char* dst, size_t len) override;
    void skip(const size_t n) override;

    using RecordReader::read;

  private:
    const char* buffer_{nullptr};
    size_t len_{0};
    const char* end_{nullptr};
};

class FileRecordReader : public RecordReader {
  public:
    FileRecordReader(const std::string& path);

    uint32_t next_record_size() override;
    void read(char* dst, size_t len) override;
    void skip(const size_t n) override;

    using RecordReader::read;

  private:
    std::ifstream fin_{};
    uint32_t next_size_{0};
};

class LiteRecordWriter {
  public:
    LiteRecordWriter(const std::string& filename)
        : fout_(filename, std::ios::binary | std::ios::trunc) {}

    void write(const char* buf, const uint32_t len);
    void write(const std::string& str) { write(str.data(), str.length()); }

    void write_raw(const std::string& str);

    template <class T>
    void write(const T& t);

    template <class T>
    void write_at(const size_t offset, const T& t);

    size_t offset() { return fout_.tellp(); }

  private:
    std::ofstream fout_{};
};

template <class T>
void LiteRecordWriter::write(const T& t) {
    const uint32_t len = sizeof(T);
    write(reinterpret_cast<const char*>(&t), len);
}

template <class T>
void LiteRecordWriter::write_at(const size_t offset, const T& t) {
    const auto cur = fout_.tellp();
    fout_.seekp(offset);
    write(t);
    fout_.seekp(max(cur, fout_.tellp()));
}

#endif /* PBRT_MESSAGES_LITE_H */
