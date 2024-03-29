#ifndef PBRT_ACCELERATORS_CLOUD_BVH_H
#define PBRT_ACCELERATORS_CLOUD_BVH_H

#include <deque>
#include <istream>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "material.h"
#include "paramset.h"
#include "pbrt.h"
#include "pbrt/raystate.h"
#include "primitive.h"
#include "texture.h"
#include "transform.h"

namespace pbrt {

struct TreeletNode;
class TriangleMesh;

class PlaceholderMaterial : public Material {
  public:
    PlaceholderMaterial(const MaterialKey &material) : material_key(material) {}

    void ComputeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                    TransportMode mode,
                                    bool allowMultipleLobes) const {
        throw std::runtime_error(
            "PlaceholderMaterial::ComputeScatteringFunctions: not implemented");
    }

    MaterialKey GetMaterialKey() const { return material_key; }
    MaterialType GetType() const { return MaterialType::Placeholder; }

  private:
    MaterialKey material_key;
};

class CloudBVH : public Aggregate {
  public:
    CloudBVH(const uint32_t bvh_root, const bool preload_all,
             const std::vector<std::shared_ptr<pbrt::Light>> *lights = nullptr);
    ~CloudBVH();

    // disallow copying
    CloudBVH(const CloudBVH &) = delete;
    CloudBVH &operator=(const CloudBVH &) = delete;

    Bounds3f WorldBound() const;

    bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool IntersectP(const Ray &ray) const;

    bool Intersect(const Ray &ray, SurfaceInteraction *isect, uint32_t) const;
    bool IntersectP(const Ray &ray, uint32_t) const;

    void Trace(RayState &rayState) const;

    void LoadTreelet(const uint32_t root_id, const char *buffer = nullptr,
                     const size_t length = 0);

    const Material *GetMaterial(const uint32_t material_id) const;

    struct TreeletNode {
        Bounds3f bounds{};
        uint8_t axis{};

        union {
            struct {
                uint16_t child_treelet[2];
                uint32_t child_node[2];
            };
            struct {
                uint32_t leaf_tag{0};
                uint32_t primitive_offset{0};
                uint32_t primitive_count{0};
            };
        };

        TreeletNode() {}

        TreeletNode(const Bounds3f &bounds, const uint8_t axis)
            : bounds(bounds), axis(axis) {}

        bool is_leaf() const { return leaf_tag == ~0; }
    };

  private:
    enum Child { LEFT = 0, RIGHT = 1 };

    struct UnfinishedTransformedPrimitive {
        size_t primitive_index;
        uint16_t instance_group;
        AnimatedTransform primitive_to_world;

        UnfinishedTransformedPrimitive(const size_t primitive_index,
                                       const uint16_t instance_group,
                                       AnimatedTransform &&primitive_to_world)
            : primitive_index(primitive_index),
              instance_group(instance_group),
              primitive_to_world(std::move(primitive_to_world)) {}
    };

    struct UnfinishedGeometricPrimitive {
        size_t primitive_index;
        MaterialKey material_key;
        uint32_t area_light_id;
        std::unique_ptr<Shape> shape;
        size_t triangle_idx;

        UnfinishedGeometricPrimitive(const size_t primitive_index,
                                     const MaterialKey &material_key,
                                     const uint32_t area_light_id,
                                     std::unique_ptr<Shape> &&shape,
                                     const size_t triangle_idx)
            : primitive_index(primitive_index),
              material_key(material_key),
              area_light_id(area_light_id),
              shape(std::move(shape)),
              triangle_idx(triangle_idx) {}
    };

    struct Treelet {
        std::map<uint32_t, std::shared_ptr<Material>> included_material{};

        std::vector<TreeletNode> nodes{};
        std::vector<std::unique_ptr<Primitive>> primitives{};
        std::vector<std::unique_ptr<Transform>> transforms{};
        std::map<uint32_t, std::shared_ptr<Primitive>> instances{};

        std::set<MaterialKey> required_materials{};
        std::set<uint16_t> required_instances{};

        std::vector<std::unique_ptr<UnfinishedTransformedPrimitive>>
            unfinished_transformed{};
        std::vector<std::unique_ptr<UnfinishedGeometricPrimitive>>
            unfinished_geometric{};
    };

    class IncludedInstance : public Aggregate {
      public:
        IncludedInstance(const Treelet *treelet, int nodeIdx)
            : treelet_(treelet), nodeIdx_(nodeIdx) {}

        Bounds3f WorldBound() const;
        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
        bool IntersectP(const Ray &ray) const;

      private:
        const Treelet *treelet_;
        int nodeIdx_;
    };

    class ExternalInstance : public Aggregate {
      public:
        ExternalInstance(const CloudBVH &bvh, uint32_t root_id)
            : bvh_(bvh), root_id_(root_id) {}

        Bounds3f WorldBound() const { return bvh_.WorldBound(); }

        bool Intersect(const Ray &ray, SurfaceInteraction *isect) const {
            return bvh_.Intersect(ray, isect, root_id_);
        }

        bool IntersectP(const Ray &ray) const {
            return bvh_.IntersectP(ray, root_id_);
        }

        uint32_t RootID() { return root_id_; }

      private:
        uint32_t root_id_;
        const CloudBVH &bvh_;
    };

    const uint32_t bvh_root_;
    bool preloading_done_{false};

    Transform identity_transform_{};
    std::shared_ptr<Texture<Float>> zero_alpha_texture_;

    std::vector<std::unique_ptr<Treelet>> treelets_{};
    std::unordered_map<uint16_t, std::shared_ptr<Primitive>> bvh_instances_{};
    std::unordered_map<uint32_t, std::shared_ptr<Material>> materials_{};
    std::map<uint32_t, std::pair<ParamSet, Transform>> area_light_params_{};
    std::vector<pbrt::Light *> scene_lights_{};

    void finalizeTreeletLoad(const uint32_t root_id);
    void loadTreeletBase(const uint32_t root_id, const char *buffer = nullptr,
                         size_t length = 0);
    void checkIfTreeletIsLoaded(const uint32_t root_id) const;
};

std::shared_ptr<CloudBVH> CreateCloudBVH(
    const ParamSet &ps, const std::vector<std::shared_ptr<Light>> &lights);

Vector3f ComputeRayDir(unsigned idx);
unsigned ComputeIdx(const Vector3f &dir);

}  // namespace pbrt

#endif /* PBRT_ACCELERATORS_CLOUD_BVH_H */
