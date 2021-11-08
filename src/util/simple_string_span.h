#ifndef PBRT_UTIL_SIMPLE_STRING_SPAN_H
#define PBRT_UTIL_SIMPLE_STRING_SPAN_H

#include <cstring>
#include <string_view>

namespace pbrt {

class string_view {
  private:
    const char* ptr;
    size_t len;

  public:
    string_view(const char* ptr, const size_t len) : ptr(ptr), len(len) {}
    const char* data() const { return ptr; }
    size_t size() const { return len; }
    size_t length() const { return size(); }
    bool empty() const { return len == 0; }
};

class simple_string_span : public string_view {
  public:
    using string_view::string_view;

    simple_string_span(string_view sv) : string_view(sv) {}

    char* mutable_data() { return const_cast<char*>(data()); }

    size_t copy(const string_view other) {
        const size_t amount_to_copy = std::min(size(), other.size());
        memcpy(mutable_data(), other.data(), amount_to_copy);
        return amount_to_copy;
    }
};

}  // namespace pbrt

#endif /* PBRT_UTIL_SIMPLE_STRING_SPAN_H */
