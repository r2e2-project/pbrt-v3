#ifndef PBRT_CLOUD_PIMAGE_H
#define PBRT_CLOUD_PIMAGE_H

#include <vector>

#include "geometry.h"
#include "mipmap.h"
#include "pbrt.h"

namespace pbrt {

class ImagePartition {
  private:
    const Point2i resolution{};
    const size_t partition_count{};
    const size_t partition_idx{};
    int padding{1};
    int x0{0}, y0{0}, w{0}, h{0};
    int W{0}, H{0};

    std::unique_ptr<RGBSpectrum[]> data{};

    const RGBSpectrum &Texel(int s, int t) const;

  public:
    ImagePartition(const Point2i &resolution, const RGBSpectrum *data,
                   const size_t partition_count, const size_t partition_idx,
                   const ImageWrap wrap_mode);

    ImagePartition(const Point2i &resolution, const size_t partition_count,
                   const size_t partition_idx, const int padding,
                   std::unique_ptr<RGBSpectrum[]> &&partition_data);

    RGBSpectrum Lookup(const Point2f &st) const;
    void WriteImage(const std::string &filename) const;
};

class PartitionedImage {
  private:
    const Point2i resolution;
    const size_t partition_count;
    const ImageWrap wrap_mode;
    std::vector<ImagePartition> partitions;

    int x_count{}, y_count{};
    int w{}, h{};

  public:
    PartitionedImage(const Point2i &resolution, const RGBSpectrum *data,
                     const size_t partition_count, const ImageWrap wrap_mode);

    RGBSpectrum Lookup(const Point2f &st) const;
    size_t GetPartitionId(const Point2f &st, bool &is_black) const;

    const ImagePartition &GetPartition(const size_t i) const {
        return partitions.at(i);
    }

    ImagePartition &GetPartition(const size_t i) { return partitions.at(i); }

    int Width() const { return w; }
    int Height() const { return h; }
};

}  // namespace pbrt

#endif /* PBRT_CLOUD_PIMAGE_H */
