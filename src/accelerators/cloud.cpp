#include "cloud.h"

#include <fstream>
#include <memory>
#include <stack>
#include <thread>

#include "bvh.h"
#include "cloud/manager.h"
#include "cloud/pimage.h"
#include "core/parallel.h"
#include "core/paramset.h"
#include "core/primitive.h"
#include "imageio.h"
#include "lights/diffuse.h"
#include "materials/matte.h"
#include "messages/compressed.h"
#include "messages/lite.h"
#include "messages/serdes.h"
#include "messages/serialization.h"
#include "messages/utils.h"
#include "pbrt.pb.h"
#include "shapes/triangle.h"

using namespace std;

namespace pbrt {

static auto &_manager = global::manager;

STAT_COUNTER("BVH/Total nodes", nNodes);
STAT_COUNTER("BVH/Visited nodes", nNodesVisited);
STAT_COUNTER("BVH/Visited primitives", nPrimitivesVisited);

struct membuf : streambuf {
    membuf(char *begin, char *end) { this->setg(begin, begin, end); }
};

CloudBVH::CloudBVH(const uint32_t bvh_root, const bool preload_all,
                   const vector<shared_ptr<Light>> *lights)
    : bvh_root_(bvh_root) {
    ProfilePhase _(Prof::AccelConstruction);

    if (lights) {
        for (auto &light : *lights) {
            scene_lights_.push_back(light.get());
        }
    }

    if (MaxThreadIndex() > 1 && !preload_all) {
        throw runtime_error(
            "Cannot use lazy-loading CloudBVH with multiple threads");
    }

    // let's load all the area lights in the case that they are used by our
    // meshes
    auto reader = _manager.GetReader(ObjectType::AreaLights);
    while (!reader->eof()) {
        protobuf::AreaLight proto;
        reader->read(&proto);
        area_light_params_.emplace(
            proto.id(),
            make_pair(from_protobuf(proto.light().paramset()),
                      from_protobuf(proto.light().light_to_world())));
    }

    if (preload_all) {
        /* (1) load all the treelets in parallel */
        const auto treelet_count = _manager.treeletCount();

        treelets_.resize(treelet_count + 1);

        ParallelFor([&](int64_t treelet_id) { loadTreeletBase(treelet_id); },
                    treelet_count);

        /* (2.A) load all the necessary materials */
        set<MaterialKey> required_materials;

        for (size_t i = 0; i < treelet_count; i++) {
            required_materials.insert(treelets_[i]->required_materials.begin(),
                                      treelets_[i]->required_materials.end());
        }

        for (const auto mkey : required_materials) {
            if (not mkey.id) {
                materials_[mkey.id] = nullptr;
                continue;
            }

            materials_[mkey.id] =
                treelets_[mkey.treelet]->included_material[mkey.id];
        }

        /* (2.B) create all the necessary external instances */
        set<uint64_t> required_instances;

        for (size_t i = 0; i < treelet_count; i++) {
            required_instances.insert(treelets_[i]->required_instances.begin(),
                                      treelets_[i]->required_instances.end());
        }

        for (const auto rid : required_instances) {
            if (not bvh_instances_.count(rid)) {
                bvh_instances_[rid] =
                    make_shared<ExternalInstance>(*this, (uint16_t)(rid >> 32));
            }
        }

        /* (3) finish loading the treelets */
        ParallelFor(
            [&](int64_t treelet_id) { finalizeTreeletLoad(treelet_id); },
            treelet_count);

        preloading_done_ = true;
    }
}

void CloudBVH::checkIfTreeletIsLoaded(const uint32_t root_id) const {
    if (preloading_done_ or
        (treelets_.size() > root_id && treelets_.at(root_id) != nullptr)) {
        return; /* this tree is already loaded */
    }

    throw runtime_error("treelet " + to_string(root_id) + " is not loaded");
}

const Material *CloudBVH::GetMaterial(const uint32_t material_id) const {
    return treelets_.at(bvh_root_)->included_material.at(material_id).get();
}

Bounds3f CloudBVH::WorldBound() const {
    // The correctness of this function is only guaranteed for the root treelet
    CHECK_EQ(bvh_root_, 0);

    checkIfTreeletIsLoaded(bvh_root_);
    return treelets_[bvh_root_]->nodes[0].bounds;
}

// Sums the full surface area for each root. Does not account for overlap
// between roots
Float CloudBVH::RootSurfaceAreas(Transform txfm) const {
    checkIfTreeletIsLoaded(bvh_root_);
    CHECK_EQ(treelets_.size(), 1);

    Float area = 0;

    vector<Bounds3f> roots;

    for (const TreeletNode &node : treelets_[bvh_root_]->nodes) {
        auto cur = txfm(node.bounds);

        bool newRoot = true;
        for (const Bounds3f &root : roots) {
            auto u = Union(root, cur);
            if (u == root) {
                newRoot = false;
                break;
            }
        }

        if (newRoot) {
            roots.push_back(cur);
            area += cur.SurfaceArea();
        }
    }

    return area;
}

Float CloudBVH::SurfaceAreaUnion() const {
    checkIfTreeletIsLoaded(bvh_root_);
    CHECK_EQ(treelets_.size(), 1);

    Bounds3f boundUnion;
    for (const TreeletNode &node : treelets_[bvh_root_]->nodes) {
        boundUnion = Union(boundUnion, node.bounds);
    }

    return boundUnion.SurfaceArea();
}

void CloudBVH::LoadTreelet(const uint32_t root_id, const char *buffer,
                           const size_t length) {
    if (preloading_done_ or
        (treelets_.size() > root_id && treelets_[root_id] != nullptr)) {
        return; /* this tree is already loaded */
    }

    if (treelets_.size() <= root_id) {
        treelets_.resize(root_id + 1);
    }

    loadTreeletBase(root_id, buffer, length);

    auto &treelet = *treelets_[root_id];

    /* create the placeholder materials */
    for (const auto mkey : treelet.required_materials) {
        materials_[mkey.id] = make_shared<PlaceholderMaterial>(mkey);
    }

    /* create the instances */
    for (const auto rid : treelet.required_instances) {
        if (not bvh_instances_.count(rid)) {
            bvh_instances_[rid] =
                make_shared<ExternalInstance>(*this, (uint16_t)(rid >> 32));
        }
    }

    finalizeTreeletLoad(root_id);
}

void CloudBVH::finalizeTreeletLoad(const uint32_t root_id) {
    auto &treelet = *treelets_[root_id];

    /* fill in unfinished primitives */
    for (auto &u : treelet.unfinished_transformed) {
        treelet.primitives[u.primitive_index] =
            make_unique<TransformedPrimitive>(bvh_instances_.at(u.instance_ref),
                                              move(u.primitive_to_world));
    }

    MediumInterface medium_interface{};

    for (auto &ug : treelet.unfinished_geometric) {
        /* do we need to make an area light for this guy? */
        shared_ptr<AreaLight> area_light;

        auto &u = *ug;
        if (u.area_light_id) {
            if (scene_lights_.empty()) {
                auto &light_data = area_light_params_.at(u.area_light_id);
                area_light = CreateDiffuseAreaLight(light_data.second,
                                                    medium_interface.outside,
                                                    light_data.first, u.shape);
                area_light->SetID(u.area_light_id + u.triangle_idx);
            } else {
                area_light = shared_ptr<AreaLight>(
                    dynamic_cast<AreaLight *>(
                        scene_lights_.at(u.area_light_id - 1)),
                    [](auto p) {});
            }
        }

        treelet.primitives[u.primitive_index] = make_unique<GeometricPrimitive>(
            u.shape, materials_.at(u.material_key.id), area_light,
            medium_interface);
    }

    treelet.required_instances.clear();
    treelet.required_materials.clear();
    treelet.unfinished_geometric.clear();
    treelet.unfinished_transformed.clear();
}

void CloudBVH::loadTreeletBase(const uint32_t root_id, const char *buffer,
                               size_t length) {
    ProfilePhase _(Prof::LoadTreelet);

    treelets_[root_id] = make_unique<Treelet>();

    auto &treelet = *treelets_[root_id];
    auto &nodes = treelet.nodes;
    auto &tree_primitives = treelet.primitives;
    auto &tree_transforms = treelet.transforms;
    auto &tree_instances = treelet.instances;

    vector<char> treelet_buffer;
    if (!buffer) {
        const string treelet_path =
            _manager.getScenePath() + "/" +
            _manager.getFileName(ObjectType::Treelet, root_id);

        ifstream fin{treelet_path, ios::binary | ios::ate};

        if (!fin.good()) {
            throw runtime_error("Could not open treelet file: " + treelet_path);
        }

        streamsize size = fin.tellg();
        fin.seekg(0, ios::beg);

        treelet_buffer.resize(size);
        fin.read(treelet_buffer.data(), size);

        buffer = treelet_buffer.data();
        length = treelet_buffer.size();
    }

    unique_ptr<RecordReader> reader;
    if (*reinterpret_cast<const uint32_t *>(buffer) == 0x184D2204) {
        reader = make_unique<CompressedReader>(buffer, length);
    } else {
        reader = make_unique<LiteRecordReader>(buffer, length);
    }

    /* read in the textures & materials included in this treelet */

    // IMAGE PARTITIONS
    const auto included_image_partitions = reader->read<uint32_t>();
    for (size_t i = 0; i < included_image_partitions; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const size_t len = reader->next_record_size();

        shared_ptr<char> image{new char[len], default_delete<char[]>()};
        reader->read(image.get(), len);

        ImagePartition partition{image};
        _manager.addInMemoryImagePartition(id, move(partition));
    }

    // PTEX TEXTURES
    const auto included_texture_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_texture_count; i++) {
        const uint32_t id = reader->read<uint32_t>();

        const size_t len = reader->next_record_size();
        shared_ptr<char> storage{new char[len], default_delete<char[]>()};
        reader->read(storage.get(), len);

        _manager.addInMemoryTexture(
            _manager.getFileName(ObjectType::Texture, id), move(storage), len);
    }

    std::map<uint64_t, std::shared_ptr<Texture<Float>>> ftexes;
    std::map<uint64_t, std::shared_ptr<Texture<Spectrum>>> stexes;

    // SPECTRUM TEXTURES
    const uint32_t included_spectrum_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_spectrum_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const string data = reader->read<string>();
        protobuf::SpectrumTexture stex_proto;
        stex_proto.ParseFromString(data);
        stexes.emplace(id, spectrum_texture::from_protobuf(stex_proto));
    }

    // FLOAT TEXTURES
    const uint32_t included_float_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_float_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const string data = reader->read<string>();
        protobuf::FloatTexture ftex_proto;
        ftex_proto.ParseFromString(data);
        ftexes.emplace(id, float_texture::from_protobuf(ftex_proto));
    }

    // MATERIALS
    const uint32_t included_material_count = reader->read<uint32_t>();
    for (size_t i = 0; i < included_material_count; i++) {
        const uint32_t id = reader->read<uint32_t>();
        const string data = reader->read<string>();
        protobuf::Material material;
        material.ParseFromString(data);
        treelet.included_material.emplace(
            id, material::from_protobuf(material, ftexes, stexes));
    }

    map<uint32_t, shared_ptr<TriangleMesh>> tree_meshes;
    map<uint32_t, MaterialKey> mesh_material_ids;
    map<uint32_t, uint32_t> mesh_area_light_id;

    /* read in the triangle meshes for this treelet */
    const uint32_t num_triangle_meshes = reader->read<uint32_t>();

    // find the start and the end of the buffer for meshes
    for (int i = 0; i < num_triangle_meshes; ++i) {
        MaterialKey material_key;

        const uint64_t tm_id = reader->read<uint64_t>();
        reader->read(&material_key);
        const uint32_t area_light_id = reader->read<uint32_t>();

        const size_t len = reader->next_record_size();
        shared_ptr<char> storage{new char[len], default_delete<char[]>()};
        reader->read(storage.get(), len);

        tree_meshes.emplace(tm_id, make_shared<TriangleMesh>(storage, 0));
        mesh_material_ids[tm_id] = material_key;

        if (area_light_id) {
            tree_meshes.at(tm_id)->alphaMask = zero_alpha_texture_;
            mesh_area_light_id[tm_id] = area_light_id;
        }
    }

    const uint32_t node_count = reader->read<uint32_t>();
    const uint32_t primitive_count = reader->read<uint32_t>();

    if (node_count == 0) {
        return;
    }

    nodes.resize(node_count);
    tree_primitives.reserve(primitive_count);

    reader->read(reinterpret_cast<char *>(&nodes[0]),
                 node_count * sizeof(TreeletNode));

    for (auto &node : nodes) {
        serdes::cloudbvh::TransformedPrimitive serdes_primitive;
        serdes::cloudbvh::Triangle serdes_triangle;

        const uint32_t transformed_primitives_count = reader->read<uint32_t>();
        const uint32_t triangles_count = reader->read<uint32_t>();

        for (int i = 0; i < transformed_primitives_count; i++) {
            reader->read(&serdes_primitive);

            tree_transforms.push_back(
                move(make_unique<Transform>(serdes_primitive.start_transform)));
            const Transform *start = tree_transforms.back().get();

            const Transform *end;
            if (start->GetMatrix() != serdes_primitive.end_transform) {
                tree_transforms.push_back(move(
                    make_unique<Transform>(serdes_primitive.end_transform)));
                end = tree_transforms.back().get();
            } else {
                end = start;
            }

            AnimatedTransform primitive_to_world{
                start, serdes_primitive.start_time, end,
                serdes_primitive.end_time};

            uint64_t instance_ref = serdes_primitive.root_ref;

            uint16_t instance_group = (uint16_t)(instance_ref >> 32);
            uint32_t instance_node = (uint32_t)instance_ref;

            if (instance_group == root_id) {
                if (not tree_instances.count(instance_ref)) {
                    tree_instances[instance_ref] =
                        make_shared<IncludedInstance>(&treelet, instance_node);
                }

                tree_primitives.push_back(make_unique<TransformedPrimitive>(
                    tree_instances[instance_ref], primitive_to_world));
            } else {
                treelet.required_instances.insert(instance_ref);

                treelet.unfinished_transformed.emplace_back(
                    tree_primitives.size(), instance_ref,
                    move(primitive_to_world));

                tree_primitives.push_back(nullptr);
            }
        }

        for (int i = 0; i < triangles_count; i++) {
            reader->read(&serdes_triangle);

            const auto mesh_id = serdes_triangle.mesh_id;
            const auto tri_number = serdes_triangle.tri_number;
            const auto material_key = mesh_material_ids[mesh_id];
            const auto area_light_id = mesh_area_light_id.count(mesh_id)
                                           ? mesh_area_light_id.at(mesh_id)
                                           : 0;

            treelet.required_materials.insert(material_key);

            auto shape = make_shared<Triangle>(
                &identity_transform_, &identity_transform_, false,
                tree_meshes.at(mesh_id), tri_number);

            treelet.unfinished_geometric.push_back(
                make_unique<UnfinishedGeometricPrimitive>(
                    tree_primitives.size(), material_key, area_light_id,
                    move(shape), i));

            tree_primitives.push_back(nullptr);
        }

        nNodes++;
    }
}

void CloudBVH::Trace(RayState &rayState) const {
    SurfaceInteraction isect;

    RayDifferential ray = rayState.ray;
    Vector3f invDir{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};

    const uint32_t currentTreelet = rayState.toVisitTop().treelet;
    checkIfTreeletIsLoaded(
        currentTreelet); /* we don't load any other treelets */

    bool hasTransform = false;
    bool transformChanged = false;

    while (true) {
        auto &top = rayState.toVisitTop();
        if (currentTreelet != top.treelet) {
            break;
        }

        RayState::TreeletNode current = move(top);
        rayState.toVisitPop();
        nNodesVisited++;

        auto &treelet = *treelets_[current.treelet];
        auto &node = treelet.nodes[current.node];

        /* prepare the ray */
        if (current.transformed != hasTransform || transformChanged) {
            transformChanged = false;

            ray = current.transformed
                      ? Inverse(rayState.rayTransform)(rayState.ray)
                      : rayState.ray;

            invDir = Vector3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
            dirIsNeg[0] = invDir.x < 0;
            dirIsNeg[1] = invDir.y < 0;
            dirIsNeg[2] = invDir.z < 0;
        }

        hasTransform = current.transformed;

        // Check ray against BVH node
        if (node.bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node.is_leaf()) {
                auto &primitives = treelet.primitives;

                for (int i = node.primitive_offset + current.primitive;
                     i < node.primitive_offset + node.primitive_count; i++) {
                    nPrimitivesVisited++;

                    if (primitives[i]->GetType() ==
                        PrimitiveType::Transformed) {
                        TransformedPrimitive *tp =
                            dynamic_cast<TransformedPrimitive *>(
                                primitives[i].get());

                        ExternalInstance *cbvh =
                            dynamic_cast<ExternalInstance *>(
                                tp->GetPrimitive().get());

                        if (cbvh) {
                            if (current.primitive + 1 < node.primitive_count) {
                                RayState::TreeletNode next_primitive = current;
                                next_primitive.primitive++;
                                rayState.toVisitPush(move(next_primitive));
                            }

                            Transform txfm;
                            tp->GetTransform().Interpolate(ray.time, &txfm);

                            RayState::TreeletNode next;
                            next.treelet = cbvh->RootID();
                            next.node = 0;

                            if (txfm.IsIdentity()) {
                                next.transformed = false;
                            } else {
                                rayState.rayTransform = txfm;
                                next.transformed = true;
                            }
                            rayState.toVisitPush(move(next));
                            break;
                        }

                        IncludedInstance *included =
                            dynamic_cast<IncludedInstance *>(
                                tp->GetPrimitive().get());
                        if (included) {
                            if (tp->Intersect(ray, &isect)) {
                                if (isect.primitive->GetMaterial()->GetType() !=
                                    MaterialType::Placeholder) {
                                    throw runtime_error(
                                        "Trace() only works with placeholder "
                                        "material");
                                }

                                const auto mat_key =
                                    dynamic_cast<const PlaceholderMaterial *>(
                                        isect.primitive->GetMaterial())
                                        ->GetMaterialKey();

                                const auto arealight =
                                    isect.primitive->GetAreaLight()
                                        ? isect.primitive->GetAreaLight()
                                              ->GetID()
                                        : 0;

                                rayState.ray.tMax = ray.tMax;
                                rayState.SetHit(current, isect, mat_key,
                                                arealight);
                            }
                        }
                    } else if (primitives[i]->Intersect(ray, &isect)) {
                        if (isect.primitive->GetMaterial()->GetType() !=
                            MaterialType::Placeholder) {
                            throw runtime_error(
                                "Trace() only works with placeholder "
                                "material");
                        }

                        const auto mat_key =
                            dynamic_cast<const PlaceholderMaterial *>(
                                isect.primitive->GetMaterial())
                                ->GetMaterialKey();

                        const auto arealight =
                            isect.primitive->GetAreaLight()
                                ? isect.primitive->GetAreaLight()->GetID()
                                : 0;

                        rayState.ray.tMax = ray.tMax;
                        rayState.SetHit(current, isect, mat_key, arealight);
                    }

                    current.primitive++;
                }

                if (rayState.toVisitEmpty()) break;
            } else {
                RayState::TreeletNode children[2];
                for (int i = 0; i < 2; i++) {
                    children[i].treelet = node.child_treelet[i];
                    children[i].node = node.child_node[i];
                    children[i].transformed = current.transformed;
                }

                if (dirIsNeg[node.axis]) {
                    rayState.toVisitPush(move(children[LEFT]));
                    rayState.toVisitPush(move(children[RIGHT]));
                } else {
                    rayState.toVisitPush(move(children[RIGHT]));
                    rayState.toVisitPush(move(children[LEFT]));
                }
            }
        } else {
            if (rayState.toVisitEmpty()) break;
        }
    }
}

bool CloudBVH::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
    return Intersect(ray, isect, bvh_root_);
}

bool CloudBVH::Intersect(const Ray &ray, SurfaceInteraction *isect,
                         const uint32_t bvh_root) const {
    ProfilePhase _(Prof::AccelIntersect);

    bool hit = false;
    Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};

    // Follow ray through BVH nodes to find primitive intersections
    pair<uint32_t, uint32_t> toVisit[64];
    uint8_t toVisitOffset = 0;

    uint32_t startTreelet = bvh_root;
    if (bvh_root == 0) {
        startTreelet = ComputeIdx(ray.d);
    }

    pair<uint32_t, uint32_t> current(startTreelet, 0);

    uint32_t prevTreelet = startTreelet;
    while (true) {
        checkIfTreeletIsLoaded(current.first);
        auto &treelet = *treelets_[current.first];
        auto &node = treelet.nodes[current.second];

        // Check ray against BVH node
        if (node.bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node.is_leaf()) {
                auto &primitives = treelet.primitives;
                for (int i = node.primitive_offset;
                     i < node.primitive_offset + node.primitive_count; i++) {
                    if (primitives[i]->Intersect(ray, isect)) hit = true;
                }

                if (toVisitOffset == 0) break;
                current = toVisit[--toVisitOffset];
            } else {
                pair<uint32_t, uint32_t> children[2];
                for (int i = 0; i < 2; i++) {
                    children[i].first = node.child_treelet[i];
                    children[i].second = node.child_node[i];
                }

                if (dirIsNeg[node.axis]) {
                    toVisit[toVisitOffset++] = children[LEFT];
                    current = children[RIGHT];
                } else {
                    toVisit[toVisitOffset++] = children[RIGHT];
                    current = children[LEFT];
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            current = toVisit[--toVisitOffset];
        }

        prevTreelet = current.first;
    }

    return hit;
}

bool CloudBVH::IntersectP(const Ray &ray) const {
    return IntersectP(ray, bvh_root_);
}

bool CloudBVH::IntersectP(const Ray &ray, const uint32_t bvh_root) const {
    ProfilePhase _(Prof::AccelIntersectP);

    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};

    // Follow ray through BVH nodes to find primitive intersections
    uint8_t toVisitOffset = 0;
    pair<uint32_t, uint32_t> toVisit[64];

    uint32_t startTreelet = bvh_root;
    if (bvh_root == 0) {
        startTreelet = ComputeIdx(ray.d);
    }

    pair<uint32_t, uint32_t> current(startTreelet, 0);

    uint32_t prevTreelet = startTreelet;
    while (true) {
        checkIfTreeletIsLoaded(current.first);
        auto &treelet = *treelets_[current.first];
        auto &node = treelet.nodes[current.second];

        // Check ray against BVH node
        if (node.bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node.is_leaf()) {
                auto &primitives = treelet.primitives;
                for (int i = node.primitive_offset;
                     i < node.primitive_offset + node.primitive_count; i++) {
                    if (primitives[i]->IntersectP(ray)) return true;
                }

                if (toVisitOffset == 0) break;
                current = toVisit[--toVisitOffset];
            } else {
                pair<uint32_t, uint32_t> children[2];
                for (int i = 0; i < 2; i++) {
                    children[i].first = node.child_treelet[i];
                    children[i].second = node.child_node[i];
                }

                if (dirIsNeg[node.axis]) {
                    toVisit[toVisitOffset++] = children[LEFT];
                    current = children[RIGHT];
                } else {
                    toVisit[toVisitOffset++] = children[RIGHT];
                    current = children[LEFT];
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            current = toVisit[--toVisitOffset];
        }

        prevTreelet = current.first;
    }

    return false;
}

void CloudBVH::clear() {
    treelets_.clear();
    bvh_instances_.clear();
    materials_.clear();
}

shared_ptr<CloudBVH> CreateCloudBVH(
    const ParamSet &ps, const vector<shared_ptr<Light>> &scene_lights_) {
    // just to supress the warnings...
    ps.FindOneBool("preload", false);
    ps.FindOneBool("sceneaccelerator", false);

    return make_shared<CloudBVH>(0, true, &scene_lights_);
}

Bounds3f CloudBVH::IncludedInstance::WorldBound() const {
    return treelet_->nodes[nodeIdx_].bounds;
}

bool CloudBVH::IncludedInstance::Intersect(const Ray &ray,
                                           SurfaceInteraction *isect) const {
    bool hit = false;
    Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};

    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = nodeIdx_;
    int nodesToVisit[64];
    while (true) {
        const CloudBVH::TreeletNode *node = &treelet_->nodes[currentNodeIndex];
        // Check ray against BVH node
        if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node->is_leaf()) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->primitive_count; ++i)
                    if (treelet_->primitives[node->primitive_offset + i]
                            ->Intersect(ray, isect))
                        hit = true;
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = node->child_node[LEFT];
                    currentNodeIndex = node->child_node[RIGHT];
                } else {
                    nodesToVisit[toVisitOffset++] = node->child_node[RIGHT];
                    currentNodeIndex = node->child_node[LEFT];
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

bool CloudBVH::IncludedInstance::IntersectP(const Ray &ray) const {
    Vector3f invDir(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    int toVisitOffset = 0, currentNodeIndex = nodeIdx_;
    int nodesToVisit[64];

    while (true) {
        const CloudBVH::TreeletNode *node = &treelet_->nodes[currentNodeIndex];
        // Check ray against BVH node
        if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node->is_leaf()) {
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->primitive_count; ++i)
                    if (treelet_->primitives[node->primitive_offset + i]
                            ->IntersectP(ray)) {
                        return true;
                    }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = node->child_node[LEFT];
                    currentNodeIndex = node->child_node[RIGHT];
                } else {
                    nodesToVisit[toVisitOffset++] = node->child_node[RIGHT];
                    currentNodeIndex = node->child_node[LEFT];
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }

    return false;
}

Vector3f ComputeRayDir(unsigned idx) {
    unsigned x = idx & (1 << 0);
    unsigned y = idx & (1 << 1);
    unsigned z = idx & (1 << 2);

    return Vector3f(x ? 1 : -1, y ? 1 : -1, z ? 1 : -1);
}

unsigned ComputeIdx(const Vector3f &dir) {
    if (PbrtOptions.directionalTreelets) {
        return (dir.x >= 0 ? 1 : 0) + ((dir.y >= 0 ? 1 : 0) << 1) +
               ((dir.z >= 0 ? 1 : 0) << 2);
    } else {
        return 0;
    }
}

}  // namespace pbrt
