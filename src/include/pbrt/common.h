#ifndef PBRT_INCLUDE_COMMON_H
#define PBRT_INCLUDE_COMMON_H

namespace pbrt {

using TreeletId = uint32_t;
using ObjectID = size_t;

enum class ObjectType {
    Treelet,
    TriangleMesh,
    Lights,
    AreaLight,
    AreaLights,
    InfiniteLights,
    ImagePartition,
    Sampler,
    Camera,
    Scene,
    Material,
    FloatTexture,
    SpectrumTexture,
    Manifest,
    Texture,
    TreeletInfo,
    StaticAssignment,
    COUNT
};

struct ObjectKey {
    ObjectType type;
    ObjectID id;

    bool operator<(const ObjectKey& other) const {
        if (type == other.type) {
            return id < other.id;
        }
        return type < other.type;
    }

    bool operator==(const ObjectKey& other) const {
        return (id == other.id) && (type == other.type);
    }
};

struct __attribute__((packed, aligned(1))) MaterialKey {
    uint32_t treelet{0};
    uint32_t id{0};

    bool operator<(const MaterialKey& o) const {
        return (treelet == o.treelet) ? (id < o.id) : (treelet < o.treelet);
    }
};

}  // namespace pbrt

#endif /* PBRT_INCLUDE_COMMON_H */
