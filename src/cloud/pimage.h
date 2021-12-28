#ifndef PBRT_CLOUD_PIMAGE_H
#define PBRT_CLOUD_PIMAGE_H

#include <vector>

#include "geometry.h"
#include "mipmap.h"
#include "pbrt.h"

namespace pbrt {

class ImagePartition {
  private:
    Point2i resolution{};
    size_t partition_count{};
    size_t partition_idx{};
    int padding{1};
    int x0{0}, y0{0}, w{0}, h{0};
    int W{0}, H{0};

    std::shared_ptr<char> storage{};
    RGBSpectrum *data{};

    const RGBSpectrum &Texel(int s, int t) const;

  public:
    ImagePartition(const Point2i &resolution, const RGBSpectrum *data,
                   const size_t partition_count, const size_t partition_idx,
                   const ImageWrap wrap_mode);

    ImagePartition(const std::shared_ptr<char> &partition_data);

    RGBSpectrum Lookup(const Point2f &st) const;
    void WriteImage(const std::string &filename) const;
};

class PartitionedImageHelper {
  private:
    const Point2i resolution;
    const size_t partition_count;
    const ImageWrap wrap_mode;
    int x_count{}, y_count{};
    int w{}, h{};
    std::vector<uint32_t> treelet_mapping;

  public:
    PartitionedImageHelper(const Point2i &resolution,
                           const size_t partition_count,
                           const ImageWrap wrap_mode,
                           const std::vector<uint32_t> &treelet_mapping = {});

    size_t GetPartitionId(const Point2f &st, bool &is_black) const;

    uint32_t GetPartitionTreeletId(const size_t partition_id) const;

    size_t PartitionCount() const { return partition_count; }
    ImageWrap WrapMode() const { return wrap_mode; }
    int Width() const { return w; }
    int Height() const { return h; }
};

class PartitionedImage {
  private:
    PartitionedImageHelper helper;
    std::vector<ImagePartition> partitions{};

  public:
    PartitionedImage(const Point2i &resolution, const RGBSpectrum *data,
                     const size_t partition_count, const ImageWrap wrap_mode);

    PartitionedImage(const Point2i &resolution,
                     std::vector<ImagePartition> &&partitions,
                     const ImageWrap wrap_mode);

    RGBSpectrum Lookup(const Point2f &st) const;

    const ImagePartition &GetPartition(const size_t i) const {
        return partitions.at(i);
    }

    ImagePartition &GetPartition(const size_t i) { return partitions.at(i); }
};

}  // namespace pbrt

#endif /* PBRT_CLOUD_PIMAGE_H */
