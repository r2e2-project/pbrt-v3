#include "integrators/pathcluster.h"

#include <iostream>

#include "bssrdf.h"
#include "camera.h"
#include "film.h"
#include "integrator.h"
#include "interaction.h"
#include "parallel.h"
#include "paramset.h"
#include "progressreporter.h"
#include "sampler.h"
#include "sampling.h"
#include "scene.h"
#include "stats.h"

namespace pbrt {

void PathClusterIntegrator::Render(const Scene &scene) {
    Preprocess(scene, *sampler);
    // Render image tiles in parallel

    // Compute number of tiles, _nTiles_, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    {
        ParallelFor2D(
            [&](Point2i tile) {
                // Render section of image corresponding to _tile_

                // Allocate _MemoryArena_ for tile
                MemoryArena arena;

                // Get sampler instance for tile
                int seed = tile.y * nTiles.x + tile.x;
                std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

                // Compute sample bounds for tile
                int x0 = sampleBounds.pMin.x + tile.x * tileSize;
                int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
                int y0 = sampleBounds.pMin.y + tile.y * tileSize;
                int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
                Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
                LOG(INFO) << "Starting image tile " << tileBounds;

                // Get _FilmTile_ for tile
                std::unique_ptr<FilmTile> filmTile =
                    camera->film->GetFilmTile(tileBounds);

                // Loop over pixels in tile to render them
                for (Point2i pixel : tileBounds) {
                    {
                        ProfilePhase pp(Prof::StartPixel);
                        tileSampler->StartPixel(pixel);
                    }

                    // Do this check after the StartPixel() call; this keeps
                    // the usage of RNG values from (most) Samplers that use
                    // RNGs consistent, which improves reproducability /
                    // debugging.
                    if (!InsideExclusive(pixel, pixelBounds)) continue;

                    do {
                        // Initialize _CameraSample_ for current sample
                        CameraSample cameraSample =
                            tileSampler->GetCameraSample(pixel);

                        // Generate camera ray for current sample
                        RayDifferential ray;
                        Float rayWeight =
                            camera->GenerateRayDifferential(cameraSample, &ray);
                        ray.ScaleDifferentials(
                            1 / std::sqrt((Float)tileSampler->samplesPerPixel));

                        // Evaluate radiance along camera ray
                        Spectrum L(0.f);
                        if (rayWeight > 0)
                            L = Li(ray, scene, *tileSampler, arena, 0);

                        // Issue warning if unexpected radiance value returned
                        if (L.HasNaNs()) {
                            LOG(ERROR) << StringPrintf(
                                "Not-a-number radiance value returned "
                                "for pixel (%d, %d), sample %d. Setting to "
                                "black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        } else if (L.y() < -1e-5) {
                            LOG(ERROR) << StringPrintf(
                                "Negative luminance value, %f, returned "
                                "for pixel (%d, %d), sample %d. Setting to "
                                "black.",
                                L.y(), pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        } else if (std::isinf(L.y())) {
                            LOG(ERROR) << StringPrintf(
                                "Infinite luminance value returned "
                                "for pixel (%d, %d), sample %d. Setting to "
                                "black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        }
                        VLOG(1) << "Camera sample: " << cameraSample
                                << " -> ray: " << ray << " -> L = " << L;

                        // Add camera ray's contribution to image
                        filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

                        // Free _MemoryArena_ memory from computing image sample
                        // value
                        arena.Reset();
                    } while (tileSampler->StartNextSample());
                }
                LOG(INFO) << "Finished image tile " << tileBounds;

                // Merge image tile into _Film_
                camera->film->MergeFilmTile(std::move(filmTile));
                reporter.Update();
            },
            nTiles);
        reporter.Done();
    }
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    camera->film->WriteImage();
}

PathIntegrator *CreatePathClusterIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    std::string lightStrategy =
        params.FindOneString("lightsamplestrategy", "spatial");
    return new PathClusterIntegrator(maxDepth, camera, sampler, pixelBounds,
                                     rrThreshold, lightStrategy);
}

}  // namespace pbrt
