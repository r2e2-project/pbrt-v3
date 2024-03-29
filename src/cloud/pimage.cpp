#include "pimage.h"

#include <fstream>

#include "imageio.h"

using namespace std;
using namespace pbrt;

PartitionedImageHelper::PartitionedImageHelper(
    const Point2i &resolution, const size_t partition_count,
    const ImageWrap wrap_mode, const std::vector<uint32_t> &treelet_mapping)
    : resolution(resolution),
      partition_count(partition_count),
      wrap_mode(wrap_mode),
      treelet_mapping(treelet_mapping) {
    if (not IsPowerOf2(resolution.x) or not IsPowerOf2(resolution.y)) {
        throw runtime_error("image dimensions have to be powers of two");
    }

    if (not IsPowerOf2(partition_count)) {
        throw runtime_error("partition count has to be a power of two");
    }

    const auto iters = static_cast<size_t>(Log2(partition_count));
    x_count = static_cast<int>(pow(2, (iters + 1) / 2));
    y_count = static_cast<int>(pow(2, iters / 2));
    w = resolution.x / x_count;
    h = resolution.y / y_count;
}

size_t PartitionedImageHelper::GetPartitionId(const Point2f &st,
                                              bool &is_black) const {
    Float s = st.x * resolution.x - 0.5f;
    Float t = st.y * resolution.y - 0.5f;
    int s0 = std::floor(s), t0 = std::floor(t);

    auto actual_point = [this](int s, int t, bool &is_black) {
        is_black = false;

        switch (wrap_mode) {
        case ImageWrap::Repeat:
            return make_pair(Mod(s, resolution.x), Mod(t, resolution.y));

        case ImageWrap::Clamp:
            return make_pair(Clamp(s, 0, resolution.x - 1),
                             Clamp(t, 0, resolution.y - 1));

        case ImageWrap::Black:
        default:
            if (s < 0 || s >= resolution.x || t < 0 || t >= resolution.y) {
                is_black = true;
            }
            return make_pair(s, t);
        }
    };

    int s_max, t_max;
    s_max = t_max = numeric_limits<int>::min();

    bool is_all_black = true;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            bool b;
            auto p = actual_point(s0 + i, t0 + j, b);
            if (b) continue;
            s_max = max(s_max, p.first);
            t_max = max(t_max, p.second);
            is_all_black = false;
        }
    }

    if (is_all_black) {
        is_black = true;
        return {};
    } else {
        is_black = false;
    }

    return (s_max / w) + (t_max / h) * x_count;
}

uint32_t PartitionedImageHelper::GetPartitionTreeletId(
    const size_t partition_id) const {
    if (treelet_mapping.empty()) {
        throw runtime_error("treelet mapping not set");
    }

    return treelet_mapping.at(partition_id);
}

PartitionedImage::PartitionedImage(const Point2i &resolution,
                                   const RGBSpectrum *data,
                                   const size_t partition_count,
                                   const ImageWrap wrap_mode)
    : helper(resolution, partition_count, wrap_mode) {
    for (size_t i = 0; i < helper.PartitionCount(); i++) {
        partitions.emplace_back(resolution, data, partition_count, i,
                                wrap_mode);
    }
}

PartitionedImage::PartitionedImage(const Point2i &resolution,
                                   vector<ImagePartition> &&partitions,
                                   const ImageWrap wrap_mode)
    : helper(resolution, partitions.size(), wrap_mode),
      partitions(move(partitions)) {}

RGBSpectrum PartitionedImage::Lookup(const Point2f &st) const {
    bool is_black;
    auto p_id = helper.GetPartitionId(st, is_black);
    if (is_black) {
        return {0.f};
    }

    return partitions[p_id].Lookup(st);
}

ImagePartition::ImagePartition(const Point2i &resolution,
                               const RGBSpectrum *data_ptr,
                               const size_t partition_count,
                               const size_t partition_idx,
                               const ImageWrap wrap_mode)
    : resolution(resolution),
      partition_count(partition_count),
      partition_idx(partition_idx) {
    if (not IsPowerOf2(resolution.x) or not IsPowerOf2(resolution.y)) {
        throw runtime_error("image dimensions have to be powers of two");
    }

    if (not IsPowerOf2(partition_count)) {
        throw runtime_error("partition count has to be a power of two");
    }

    const auto iters = static_cast<size_t>(Log2(partition_count));
    const auto x_count = static_cast<size_t>(pow(2, (iters + 1) / 2));
    const auto y_count = static_cast<size_t>(pow(2, iters / 2));

    // let's find the coordinates for this partition
    w = resolution.x / x_count;
    h = resolution.y / y_count;
    x0 = (partition_idx % x_count) * w;
    y0 = (partition_idx / x_count) * h;

    W = w + 2 * padding;
    H = h + 2 * padding;

    storage = make_unique<char[]>(W * H * sizeof(RGBSpectrum));
    data = reinterpret_cast<RGBSpectrum *>(storage.get());

    // copy the main pixels
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            const auto s = x0 + i;
            const auto t = y0 + j;
            data[(i + padding) + (j + padding) * W] =
                data_ptr[s + resolution.x * t];
        }
    }

    auto get_color = [&](const int s, const int t) {
        static const RGBSpectrum black = 0.f;

        switch (wrap_mode) {
        case ImageWrap::Repeat:
            return data_ptr[Mod(s, resolution.x) +
                            resolution.x * Mod(t, resolution.y)];

        case ImageWrap::Clamp:
            return data_ptr[Clamp(s, 0, resolution.x - 1) +
                            resolution.x * Clamp(t, 0, resolution.y - 1)];

        case ImageWrap::Black:
        default:
            if (s < 0 || s >= resolution.x || t < 0 || t >= resolution.y) {
                return black;
            }

            return data_ptr[s + t * resolution.x];
        }
    };

    // copy padding based on wrap mode
    // (1) right columns
    for (int x_pad = 0; x_pad < padding; x_pad++) {
        for (int y = -padding; y < h + padding; y++) {
            const int s = x0 + w + x_pad;
            const int t = y0 + y;
            const int i = padding + w + x_pad;
            const int j = padding + y;
            data[i + W * j] = get_color(s, t);
        }
    }

    // (2) left columns
    for (int x_pad = 0; x_pad < padding; x_pad++) {
        for (int y = -padding; y < h + padding; y++) {
            const int s = x0 - x_pad - 1;
            const int t = y0 + y;
            const int i = padding - x_pad - 1;
            const int j = padding + y;
            data[i + W * j] = get_color(s, t);
        }
    }

    // (3) top rows
    for (int y_pad = 0; y_pad < padding; y_pad++) {
        for (int x = -padding; x < w + padding; x++) {
            const int s = x0 + x;
            const int t = y0 - y_pad - 1;
            const int i = padding + x;
            const int j = padding - y_pad - 1;
            data[i + W * j] = get_color(s, t);
        }
    }

    // (4) bottom rows
    for (int y_pad = 0; y_pad < padding; y_pad++) {
        for (int x = -padding; x < w + padding; x++) {
            const int s = x0 + x;
            const int t = y0 + h + y_pad;
            const int i = padding + x;
            const int j = padding + h + y_pad;
            data[i + W * j] = get_color(s, t);
        }
    }
}

ImagePartition::ImagePartition(unique_ptr<char[]> &&partition_data)
    : storage(move(partition_data)) {
    int *ptr = reinterpret_cast<int *>(storage.get());
    resolution.x = ptr[0];
    resolution.y = ptr[1];
    partition_idx = ptr[2];
    partition_count = ptr[3];
    data = reinterpret_cast<RGBSpectrum *>(storage.get() + sizeof(int) * 4);

    if (not IsPowerOf2(resolution.x) or not IsPowerOf2(resolution.y)) {
        throw runtime_error("image dimensions have to be powers of two");
    }

    if (not IsPowerOf2(partition_count)) {
        throw runtime_error("partition count has to be a power of two");
    }

    const auto iters = static_cast<size_t>(Log2(partition_count));
    const auto x_count = static_cast<size_t>(pow(2, (iters + 1) / 2));
    const auto y_count = static_cast<size_t>(pow(2, iters / 2));

    // let's find the coordinates for this partition
    w = resolution.x / x_count;
    h = resolution.y / y_count;
    x0 = (partition_idx % x_count) * w;
    y0 = (partition_idx / x_count) * h;

    W = w + 2 * padding;
    H = h + 2 * padding;
}

void ImagePartition::WriteImage(const string &filename) const {
    ofstream fout{filename, ios::trunc | ios::binary};
    fout.write(reinterpret_cast<const char *>(&resolution.x), sizeof(int));
    fout.write(reinterpret_cast<const char *>(&resolution.y), sizeof(int));
    fout.write(reinterpret_cast<const char *>(&partition_idx), sizeof(int));
    fout.write(reinterpret_cast<const char *>(&partition_count), sizeof(int));
    fout.write(reinterpret_cast<const char *>(data),
               W * H * sizeof(RGBSpectrum));
    fout.close();
}

const RGBSpectrum &ImagePartition::Texel(int s, int t) const {
    const int i = (s - x0) + padding;
    const int j = (t - y0) + padding;
    return data[i + j * W];
}

RGBSpectrum ImagePartition::Lookup(const Point2f &st) const {
    const Float s = st.x * resolution.x - 0.5f;
    const Float t = st.y * resolution.y - 0.5f;
    int s0 = std::floor(s), t0 = std::floor(t);
    const Float ds = s - s0, dt = t - t0;

    s0 = (s0 < 0) ? Mod(s0, resolution.x) : s0;
    t0 = (t0 < 0) ? Mod(t0, resolution.y) : t0;

    return (1 - ds) * (1 - dt) * Texel(s0, t0) +
           (1 - ds) * dt * Texel(s0, t0 + 1) +
           ds * (1 - dt) * Texel(s0 + 1, t0) + ds * dt * Texel(s0 + 1, t0 + 1);
}
