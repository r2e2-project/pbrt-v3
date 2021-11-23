#include "pimage.h"

#include "imageio.h"

using namespace std;
using namespace pbrt;

PartitionedImage::PartitionedImage(const Point2i &resolution,
                                   const RGBSpectrum *data,
                                   const size_t partition_count,
                                   const ImageWrap wrap_mode)
    : resolution(resolution),
      partition_count(partition_count),
      wrap_mode(wrap_mode) {
    if (not IsPowerOf2(resolution.x) or not IsPowerOf2(resolution.y)) {
        throw runtime_error("image dimensions have to be powers of two");
    }

    if (not IsPowerOf2(partition_count)) {
        throw runtime_error("partition count has to be a power of two");
    }

    size_t padding = 25;

    for (size_t i = 0; i < partition_count; i++) {
        partitions.emplace_back(resolution, data, partition_count, i, wrap_mode,
                                padding);
    }
}

RGBSpectrum PartitionedImage::Lookup(const Point2f &st) const {
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
            if (s < 0 || s >= resolution.x || t < 0 || t >= resolution.y) {
                is_black = true;
            }
            return make_pair(s, t);
        }
    };
}

ImagePartition::ImagePartition(const Point2i &resolution,
                               const RGBSpectrum *data_ptr,
                               const size_t partition_count,
                               const size_t partition_idx,
                               const ImageWrap wrap_mode, const int padding)
    : resolution(resolution),
      partition_count(partition_count),
      partition_idx(partition_idx),
      wrap_mode(wrap_mode),
      padding(padding) {
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

    data = make_unique<RGBSpectrum[]>(W * H);

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

void ImagePartition::WriteImage(const string &filename) {
    pbrt::WriteImage(filename, (Float *)data.get(),
                     {Point2i(0, 0), Point2i(W, H)}, Point2i(W, H));
}

const RGBSpectrum &ImagePartition::Texel(int s, int t) const {
    switch (wrap_mode) {
    case ImageWrap::Repeat:
        s = Mod(s, resolution.x);
        t = Mod(t, resolution.y);
        break;
    case ImageWrap::Clamp:
        s = Clamp(s, 0, resolution.x - 1);
        t = Clamp(t, 0, resolution.y - 1);
        break;
    case ImageWrap::Black: {
        static const RGBSpectrum black = 0.f;
        if (s < 0 || s >= resolution.x || t < 0 || t >= resolution.y)
            return black;
        break;
    }
    }

    return data[s + resolution.x * t];
}

RGBSpectrum ImagePartition::Lookup(const Point2f &st) const {
    Float s = st.x * resolution.x - 0.5f;
    Float t = st.y * resolution.y - 0.5f;
    int s0 = std::floor(s), t0 = std::floor(t);
    Float ds = s - s0, dt = t - t0;
    return (1 - ds) * (1 - dt) * Texel(s0, t0) +
           (1 - ds) * dt * Texel(s0, t0 + 1) +
           ds * (1 - dt) * Texel(s0 + 1, t0) + ds * dt * Texel(s0 + 1, t0 + 1);
}
