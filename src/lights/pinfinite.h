
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_LIGHTS_PINFINITE_H
#define PBRT_LIGHTS_PINFINITE_H

// lights/pinfinite.h*
#include "cloud/pimage.h"
#include "light.h"
#include "mipmap.h"
#include "pbrt.h"
#include "sampling.h"
#include "scene.h"
#include "shape.h"
#include "texture.h"

namespace pbrt {

class BasePartitionedInfiniteAreaLight : public Light {
  public:
    BasePartitionedInfiniteAreaLight(const Transform &LightToWorld,
                                     const Spectrum &power, int nSamples,
                                     const RGBSpectrum *downsampledImg,
                                     const Point2i &downsampledImgResolution);

    void Preprocess(const Scene &scene) {
        scene.WorldBound().BoundingSphere(&worldCenter, &worldRadius);
    }

    Spectrum Power() const;
    Spectrum Le(const RayDifferential &ray) const;
    Spectrum Sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi,
                       Float *pdf, VisibilityTester *vis) const;
    Spectrum Sample_Le(const Point2f &u1, const Point2f &u2, Float time,
                       Ray *ray, Normal3f *nLight, Float *pdfPos,
                       Float *pdfDir) const;

    Float Pdf_Li(const Interaction &, const Vector3f &) const;
    void Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                Float *pdfDir) const;

    Point2f Le_SampledPoint(const RayDifferential &ray) const;
    Point2f Sample_Li_SampledPoint(const Interaction &ref, const Point2f &u,
                                   Vector3f *wi, Float *pdf,
                                   VisibilityTester *vis, bool &isBlack) const;
    Point2f Sample_Le_SampledPoint(const Point2f &u1, const Point2f &u2,
                                   Float time, Ray *ray, Normal3f *nLight,
                                   Float *pdfPos, Float *pdfDir,
                                   bool &isBlack) const;

    LightType GetType() const { return LightType::PartitionedInfinite; }

  protected:
    Spectrum L;
    std::unique_ptr<MIPMap<RGBSpectrum>> downsampledLmap;
    std::unique_ptr<Distribution2D> distribution;

  private:
    Spectrum power;
    Point3f worldCenter;
    Float worldRadius;
};

class CloudInfiniteAreaLight : public BasePartitionedInfiniteAreaLight {
  public:
    CloudInfiniteAreaLight(const Transform &LightToWorld, const Spectrum &power,
                           int nSamples, const RGBSpectrum *downsampledImg,
                           const Point2i &downsampledImgResolution,
                           const Point2i &fullResolution,
                           const std::vector<uint32_t> &treeletMapping);

    std::pair<uint32_t, uint32_t> GetPointImageInfo(const Point2f &uv,
                                                    bool &isBlack) const;

  private:
    PartitionedImageHelper pImageHelper;
};

// InfiniteAreaLight Declarations
class PartitionedInfiniteAreaLight : public BasePartitionedInfiniteAreaLight {
  public:
    // InfiniteAreaLight Public Methods
    PartitionedInfiniteAreaLight(const Transform &LightToWorld,
                                 const Spectrum &power, int nSamples,
                                 PartitionedImage &&Lmap,
                                 const RGBSpectrum *downsampledImg,
                                 const Point2i &downsampledImgResolution);

    Spectrum Le(const RayDifferential &ray) const {
        const auto uv = Le_SampledPoint(ray);
        return Spectrum(Lmap.Lookup(uv) * L, SpectrumType::Illuminant);
    }

    Spectrum Sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi,
                       Float *pdf, VisibilityTester *vis) const {
        bool isBlack = false;
        const auto p = Sample_Li_SampledPoint(ref, u, wi, pdf, vis, isBlack);

        if (isBlack) {
            return Spectrum{0.f};
        } else {
            return Spectrum(Lmap.Lookup(p) * L, SpectrumType::Illuminant);
        }
    }

    Spectrum Sample_Le(const Point2f &u1, const Point2f &u2, Float time,
                       Ray *ray, Normal3f *nLight, Float *pdfPos,
                       Float *pdfDir) const {
        bool isBlack = false;
        const auto p = Sample_Le_SampledPoint(u1, u2, time, ray, nLight, pdfPos,
                                              pdfDir, isBlack);
        if (isBlack) {
            return Spectrum{0.f};
        } else {
            return Spectrum(Lmap.Lookup(p) * L, SpectrumType::Illuminant);
        }
    }

  private:
    // PartitionedInfiniteAreaLight Private Data
    PartitionedImage Lmap;
};

std::shared_ptr<PartitionedInfiniteAreaLight> CreatePartitionedInfiniteLight(
    const Transform &light2world, const ParamSet &paramSet);

}  // namespace pbrt

#endif  // PBRT_LIGHTS_PINFINITE_H
