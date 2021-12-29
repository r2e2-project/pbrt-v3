
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

// lights/infinite.cpp*
#include "lights/pinfinite.h"

#include "imageio.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

// PartitionedInfiniteAreaLight Method Definitions
BasePartitionedInfiniteAreaLight::BasePartitionedInfiniteAreaLight(
    const Transform &LightToWorld, const Spectrum &power, int nSamples,
    const RGBSpectrum *downsampledImg, const Point2i &downsampledImgResolution)
    : Light((int)LightFlags::Infinite, LightToWorld, MediumInterface(),
            nSamples),
      L(1.f),
      power(power) {
    downsampledLmap.reset(
        new MIPMap<RGBSpectrum>(downsampledImgResolution, downsampledImg));

    // Compute scalar-valued image _img_ from environment map
    int width = 2 * downsampledLmap->Width(),
        height = 2 * downsampledLmap->Height();
    std::unique_ptr<Float[]> img(new Float[width * height]);
    float fwidth = 0.5f / std::min(width, height);
    ParallelFor(
        [&](int64_t v) {
            Float vp = (v + .5f) / (Float)height;
            Float sinTheta = std::sin(Pi * (v + .5f) / height);
            for (int u = 0; u < width; ++u) {
                Float up = (u + .5f) / (Float)width;
                img[u + v * width] =
                    downsampledLmap->Lookup(Point2f(up, vp), fwidth).y();
                img[u + v * width] *= sinTheta;
            }
        },
        height, 32);

    // Compute sampling distributions for rows and columns of image
    distribution.reset(new Distribution2D(img.get(), width, height));
}

Spectrum BasePartitionedInfiniteAreaLight::Le(
    const RayDifferential &ray) const {
    const auto uv = Le_SampledPoint(ray);
    return Spectrum(downsampledLmap->Lookup(uv) * L, SpectrumType::Illuminant);
}

Spectrum BasePartitionedInfiniteAreaLight::Sample_Li(
    const Interaction &ref, const Point2f &u, Vector3f *wi, Float *pdf,
    VisibilityTester *vis) const {
    bool isBlack = false;
    const auto p = Sample_Li_SampledPoint(ref, u, wi, pdf, vis, isBlack);

    if (isBlack) {
        return Spectrum{0.f};
    } else {
        return Spectrum(downsampledLmap->Lookup(p) * L,
                        SpectrumType::Illuminant);
    }
}

Spectrum BasePartitionedInfiniteAreaLight::Sample_Le(
    const Point2f &u1, const Point2f &u2, Float time, Ray *ray,
    Normal3f *nLight, Float *pdfPos, Float *pdfDir) const {
    bool isBlack = false;
    const auto p = Sample_Le_SampledPoint(u1, u2, time, ray, nLight, pdfPos,
                                          pdfDir, isBlack);
    if (isBlack) {
        return Spectrum{0.f};
    } else {
        return Spectrum(downsampledLmap->Lookup(p) * L,
                        SpectrumType::Illuminant);
    }
}

Spectrum BasePartitionedInfiniteAreaLight::Power() const {
    return Pi * worldRadius * worldRadius * power * L;
}

Point2f BasePartitionedInfiniteAreaLight::Le_SampledPoint(
    const RayDifferential &ray) const {
    Vector3f w = Normalize(WorldToLight(ray.d));
    Point2f st(SphericalPhi(w) * Inv2Pi, SphericalTheta(w) * InvPi);
    return st;
    // return Spectrum(Lmap.Lookup(st) * L, SpectrumType::Illuminant);
}

Point2f BasePartitionedInfiniteAreaLight::Sample_Li_SampledPoint(
    const Interaction &ref, const Point2f &u, Vector3f *wi, Float *pdf,
    VisibilityTester *vis, bool &isBlack) const {
    ProfilePhase _(Prof::LightSample);

    isBlack = false;

    Float mapPdf;
    Point2f uv = distribution->SampleContinuous(u, &mapPdf);
    if (mapPdf == 0) {
        isBlack = true;
        return {};
    }

    // Convert infinite light sample point to direction
    Float theta = uv[1] * Pi, phi = uv[0] * 2 * Pi;
    Float cosTheta = std::cos(theta), sinTheta = std::sin(theta);
    Float sinPhi = std::sin(phi), cosPhi = std::cos(phi);
    *wi =
        LightToWorld(Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));

    // Compute PDF for sampled infinite light direction
    *pdf = mapPdf / (2 * Pi * Pi * sinTheta);
    if (sinTheta == 0) *pdf = 0;

    // Return radiance value for infinite light direction
    *vis = VisibilityTester(ref, Interaction(ref.p + *wi * (2 * worldRadius),
                                             ref.time, mediumInterface));
    return uv;
    // return Spectrum(Lmap.Lookup(uv) * L, SpectrumType::Illuminant);
}

Float BasePartitionedInfiniteAreaLight::Pdf_Li(const Interaction &,
                                               const Vector3f &w) const {
    ProfilePhase _(Prof::LightPdf);
    Vector3f wi = WorldToLight(w);
    Float theta = SphericalTheta(wi), phi = SphericalPhi(wi);
    Float sinTheta = std::sin(theta);
    if (sinTheta == 0) return 0;
    return distribution->Pdf(Point2f(phi * Inv2Pi, theta * InvPi)) /
           (2 * Pi * Pi * sinTheta);
}

Point2f BasePartitionedInfiniteAreaLight::Sample_Le_SampledPoint(
    const Point2f &u1, const Point2f &u2, Float time, Ray *ray,
    Normal3f *nLight, Float *pdfPos, Float *pdfDir, bool &isBlack) const {
    ProfilePhase _(Prof::LightSample);

    isBlack = false;

    Float mapPdf;
    Point2f uv = distribution->SampleContinuous(u1, &mapPdf);
    if (mapPdf == 0) {
        isBlack = true;
        return {};
    }

    Float theta = uv[1] * Pi, phi = uv[0] * 2.f * Pi;
    Float cosTheta = std::cos(theta), sinTheta = std::sin(theta);
    Float sinPhi = std::sin(phi), cosPhi = std::cos(phi);
    Vector3f d =
        -LightToWorld(Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));
    *nLight = (Normal3f)d;

    // Compute origin for infinite light sample ray
    Vector3f v1, v2;
    CoordinateSystem(-d, &v1, &v2);
    Point2f cd = ConcentricSampleDisk(u2);
    Point3f pDisk = worldCenter + worldRadius * (cd.x * v1 + cd.y * v2);
    *ray = Ray(pDisk + worldRadius * -d, d, Infinity, time);

    // Compute _PartitionedInfiniteAreaLight_ ray PDFs
    *pdfDir = sinTheta == 0 ? 0 : mapPdf / (2 * Pi * Pi * sinTheta);
    *pdfPos = 1 / (Pi * worldRadius * worldRadius);

    return uv;
    // return Spectrum(Lmap.Lookup(uv) * L, SpectrumType::Illuminant);
}

void BasePartitionedInfiniteAreaLight::Pdf_Le(const Ray &ray, const Normal3f &,
                                              Float *pdfPos,
                                              Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    Vector3f d = -WorldToLight(ray.d);
    Float theta = SphericalTheta(d), phi = SphericalPhi(d);
    Point2f uv(phi * Inv2Pi, theta * InvPi);
    Float mapPdf = distribution->Pdf(uv);
    *pdfDir = mapPdf / (2 * Pi * Pi * std::sin(theta));
    *pdfPos = 1 / (Pi * worldRadius * worldRadius);
}

CloudInfiniteAreaLight::CloudInfiniteAreaLight(
    const Transform &LightToWorld, const Spectrum &power, int nSamples,
    const RGBSpectrum *downsampledImg, const Point2i &downsampledImgResolution,
    const Point2i &fullResolution, const std::vector<uint32_t> &treeletMapping)
    : BasePartitionedInfiniteAreaLight(LightToWorld, power, nSamples,
                                       downsampledImg,
                                       downsampledImgResolution),
      pImageHelper(fullResolution, treeletMapping.size(), ImageWrap::Repeat,
                   treeletMapping) {}

std::pair<uint32_t, uint32_t> CloudInfiniteAreaLight::GetPointImageInfo(
    const Point2f &uv, bool &isBlack) const {
    const auto partition = pImageHelper.GetPartitionId(uv, isBlack);
    if (isBlack) return {0, 0};
    const auto treelet = pImageHelper.GetPartitionTreeletId(partition);
    return std::make_pair(treelet, partition);
}

PartitionedInfiniteAreaLight::PartitionedInfiniteAreaLight(
    const Transform &LightToWorld, const Spectrum &power, int nSamples,
    PartitionedImage &&Lmap, const RGBSpectrum *downsampledImg,
    const Point2i &downsampledImgResolution)
    : BasePartitionedInfiniteAreaLight(LightToWorld, power, nSamples,
                                       downsampledImg,
                                       downsampledImgResolution),
      Lmap(std::move(Lmap)) {}

std::shared_ptr<PartitionedInfiniteAreaLight> CreatePartitionedInfiniteLight(
    const Transform &light2world, const ParamSet &paramSet) {
    // Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    // Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    // std::string texmap = paramSet.FindOneFilename("mapname", "");
    // int nSamples =
    //     paramSet.FindOneInt("samples", paramSet.FindOneInt("nsamples", 1));
    // if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);
    // return std::make_shared<PartitionedInfiniteAreaLight>(light2world, L *
    // sc,
    //                                                       nSamples, texmap);
    throw std::runtime_error("no implemented");
}

}  // namespace pbrt
