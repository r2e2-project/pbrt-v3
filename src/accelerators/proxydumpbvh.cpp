#include "proxydumpbvh.h"

#include <Ptexture.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <queue>
#include <type_traits>

#include "accelerators/cloud.h"
#include "messages/lite.h"
#include "messages/serdes.h"
#include "messages/utils.h"
#include "paramset.h"
#include "pbrt.pb.h"
#include "stats.h"

using namespace std;

namespace pbrt {

static auto &_manager = global::manager;

STAT_COUNTER("BVH/Total Ray Transfers", totalRayTransfers);

namespace SizeEstimates {
constexpr uint64_t nodeSize = sizeof(CloudBVH::TreeletNode);
// triNum, faceIndex, pointer to mesh, 3 indices for triangle
// assume on average 2 unique vertices, normals etc per triangle
constexpr uint64_t triSize = sizeof(int) + sizeof(int) + sizeof(uintptr_t) +
                             3 * sizeof(int) +
                             2 * (sizeof(Point3f) + sizeof(Normal3f) +
                                  sizeof(Vector3f) + sizeof(Point2f));
constexpr uint64_t instSize = 32 * sizeof(float) + sizeof(int);
}  // namespace SizeEstimates

ProxyDumpBVH::ProxyDumpBVH(vector<shared_ptr<Primitive>> &&p,
                           int maxTreeletBytes, int copyableThreshold,
                           bool writeHeader, bool inlineProxies,
                           ProxyDumpBVH::TraversalAlgorithm travAlgo,
                           ProxyDumpBVH::PartitionAlgorithm partAlgo,
                           int maxPrimsInNode, SplitMethod splitMethod)
    : BVHAccel(p, maxPrimsInNode, splitMethod),
      traversalAlgo(travAlgo),
      partitionAlgo(partAlgo) {
    SetNodeInfo(maxTreeletBytes, copyableThreshold);
    allTreelets = AllocateTreelets(maxTreeletBytes);

    if (writeHeader) {
        DumpHeader();
    }

    if (PbrtOptions.dumpScene) {
        DumpTreelets(true, inlineProxies);
    }
}

shared_ptr<ProxyDumpBVH> CreateProxyDumpBVH(vector<shared_ptr<Primitive>> prims,
                                            const ParamSet &ps) {
    int maxTreeletBytes = ps.FindOneInt("maxtreeletbytes", 1'000'000'000);
    int copyableThreshold =
        ps.FindOneInt("copyablethreshold", maxTreeletBytes / 2);

    string travAlgoName = ps.FindOneString("traversal", "sendcheck");
    ProxyDumpBVH::TraversalAlgorithm travAlgo;
    if (travAlgoName == "sendcheck")
        travAlgo = ProxyDumpBVH::TraversalAlgorithm::SendCheck;
    else if (travAlgoName == "checksend")
        travAlgo = ProxyDumpBVH::TraversalAlgorithm::CheckSend;
    else {
        Warning("BVH traversal algorithm \"%s\" unknown. Using \"SendCheck\".",
                travAlgoName.c_str());
    }

    string partAlgoName = ps.FindOneString("partition", "topological");
    ProxyDumpBVH::PartitionAlgorithm partAlgo;
    if (partAlgoName == "topological")
        partAlgo = ProxyDumpBVH::PartitionAlgorithm::Topological;
    else if (partAlgoName == "mergedgraph")
        partAlgo = ProxyDumpBVH::PartitionAlgorithm::MergedGraph;
    else if (partAlgoName == "nvidia")
        partAlgo = ProxyDumpBVH::PartitionAlgorithm::Nvidia;
    else {
        Warning(
            "BVH partition algorithm \"%s\" unknown. Using \"Topological\".",
            partAlgoName.c_str());
    }

    bool writeHeader = ps.FindOneBool("writeheader", false);
    bool inlineProxies = ps.FindOneBool("inlineproxies", true);

    string splitMethodName = ps.FindOneString("splitmethod", "sah");
    BVHAccel::SplitMethod splitMethod;
    if (splitMethodName == "sah")
        splitMethod = BVHAccel::SplitMethod::SAH;
    else if (splitMethodName == "hlbvh")
        splitMethod = BVHAccel::SplitMethod::HLBVH;
    else if (splitMethodName == "middle")
        splitMethod = BVHAccel::SplitMethod::Middle;
    else if (splitMethodName == "equal")
        splitMethod = BVHAccel::SplitMethod::EqualCounts;
    else {
        Warning("BVH split method \"%s\" unknown.  Using \"sah\".",
                splitMethodName.c_str());
        splitMethod = BVHAccel::SplitMethod::SAH;
    }
    int maxPrimsInNode = ps.FindOneInt("maxnodeprims", 4);

    // Top level BVH should have as many prims to work with as possible
    if (inlineProxies) {
        maxPrimsInNode = 1;
    }

    return make_shared<ProxyDumpBVH>(
        move(prims), maxTreeletBytes, copyableThreshold, writeHeader,
        inlineProxies, travAlgo, partAlgo, maxPrimsInNode, splitMethod);
}

void ProxyDumpBVH::SetNodeInfo(int maxTreeletBytes, int copyableThreshold) {
    printf("Building general BVH node information\n");
    nodeSizes.resize(nodeCount);
    subtreeSizes.resize(nodeCount);
    nodeParents.resize(nodeCount);
    nodeProxies.resize(nodeCount);
    nodeUnsharedProxies.resize(nodeCount);
    subtreeProxies.resize(nodeCount);
    nodeProxySizes.resize(nodeCount);
    subtreeProxySizes.resize(nodeCount);
    nodeUnsharedProxySizes.resize(nodeCount);
    nodeBounds.resize(nodeCount);

    uint64_t totalNodeBytes = 0;
    for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
        const LinearBVHNode &node = nodes[nodeIdx];

        uint64_t totalSize = SizeEstimates::nodeSize;

        unordered_set<const ProxyBVH *> includedProxies;

        for (int primIdx = 0; primIdx < node.nPrimitives; primIdx++) {
            auto &prim = primitives[node.primitivesOffset + primIdx];
            if (prim->GetType() == PrimitiveType::Geometric) {
                totalSize += SizeEstimates::triSize;
            } else if (prim->GetType() == PrimitiveType::Transformed) {
                totalSize += SizeEstimates::instSize;

                shared_ptr<TransformedPrimitive> tp =
                    dynamic_pointer_cast<TransformedPrimitive>(prim);
                shared_ptr<ProxyBVH> proxy =
                    dynamic_pointer_cast<ProxyBVH>(tp->GetPrimitive());
                CHECK_NOTNULL(proxy.get());

                auto processDep = [&](const ProxyBVH *proxy) {
                    allProxies.emplace(proxy);

                    if (proxy->Size() > copyableThreshold) {
                        largeProxies.emplace(proxy);
                    } else {
                        if (proxy->UsageCount() == 1) {
                            totalSize += proxy->Size();
                            nodeUnsharedProxies[nodeIdx].push_back(proxy);
                            nodeUnsharedProxySizes[nodeIdx] += proxy->Size();
                        } else {
                            includedProxies.emplace(proxy);
                        }
                    }
                };

                processDep(proxy.get());

                for (const ProxyBVH *dep : proxy->Dependencies()) {
                    processDep(dep);
                }
            }
        }

        auto kv = proxySets.emplace(move(includedProxies));
        nodeProxies[nodeIdx] = &*kv.first;

        nodeSizes[nodeIdx] = totalSize;

        if (node.nPrimitives == 0) {
            nodeParents[nodeIdx + 1] = nodeIdx;
            nodeParents[node.secondChildOffset] = nodeIdx;
        }

        totalNodeBytes += nodeSizes[nodeIdx];
    }

    // Specific to NVIDIA algorithm
    const float AREA_EPSILON =
        nodes[0].bounds.SurfaceArea() * maxTreeletBytes / (totalNodeBytes * 10);

    for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
        const LinearBVHNode &node = nodes[nodeIdx];

        nodeBounds[nodeIdx] =
            nodes[nodeIdx].bounds.SurfaceArea() + AREA_EPSILON;
    }

    uint32_t proxyIdx = 0;
    for (const ProxyBVH *proxy : allProxies) {
        proxyOrder.emplace(proxy, proxyIdx);
        proxyIdx++;
    }

    auto unorderedUnion = [](const unordered_set<const ProxyBVH *> *a,
                             const unordered_set<const ProxyBVH *> *b) {
        auto setUnion = *a;
        for (const ProxyBVH *ptr : *b) {
            setUnion.emplace(ptr);
        }

        return setUnion;
    };

    for (uint64_t nodeIdx = nodeCount; nodeIdx-- > 0;) {
        const LinearBVHNode &node = nodes[nodeIdx];
        subtreeSizes[nodeIdx] = nodeSizes[nodeIdx];
        subtreeProxies[nodeIdx] = nodeProxies[nodeIdx];
        if (node.nPrimitives == 0) {
            subtreeProxies[nodeIdx] =
                ProxyUnion(subtreeProxies[nodeIdx + 1],
                           subtreeProxies[node.secondChildOffset]);

            subtreeSizes[nodeIdx] += subtreeSizes[nodeIdx + 1] +
                                     subtreeSizes[node.secondChildOffset];
        }
    }

    for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
        nodeProxySizes[nodeIdx] = GetProxyBytes(nodeProxies[nodeIdx]);
        subtreeProxySizes[nodeIdx] = GetProxyBytes(subtreeProxies[nodeIdx]);
    }

    printf("Done building general BVH node information\n");
}

uint64_t ProxyDumpBVH::GetProxyBytes(ProxySetPtr proxies) const {
    auto iter = proxySizeCache.find(proxies);
    if (iter != proxySizeCache.end()) {
        return iter->second;
    }

    uint64_t totalProxiesSize = 0;
    for (const ProxyBVH *proxy : *proxies) {
        totalProxiesSize += proxy->Size();
    }

    const_cast<unordered_map<ProxySetPtr, uint64_t> *>(&proxySizeCache)
        ->emplace(proxies, totalProxiesSize);

    return totalProxiesSize;
}

ProxyDumpBVH::ProxySetPtr ProxyDumpBVH::ProxyUnion(
    ProxyDumpBVH::ProxySetPtr a, ProxyDumpBVH::ProxySetPtr b) const {
    if (a == b) {
        return a;
    }

    if (a == nullptr) {
        return b;
    }

    if (b == nullptr) {
        return a;
    }

    auto setUnion = *a;

    for (const ProxyBVH *ptr : *b) {
        setUnion.emplace(ptr);
    }

    auto kv =
        const_cast<unordered_set<unordered_set<const ProxyBVH *>> *>(&proxySets)
            ->emplace(move(setUnion));
    return &*kv.first;
}

ProxyDumpBVH::TreeletInfo::TreeletInfo(IntermediateTreeletInfo &&info)
    : nodes(move(info.nodes)),
      proxies(),
      noProxySize(info.noProxySize),
      proxySize(info.proxySize + info.unsharedProxySize),
      dirIdx(info.dirIdx),
      totalProb(info.totalProb) {
    proxies.insert(proxies.end(), info.proxies->begin(), info.proxies->end());
    proxies.insert(proxies.end(), info.unsharedProxies.begin(),
                   info.unsharedProxies.end());
}

unordered_map<uint32_t, ProxyDumpBVH::IntermediateTreeletInfo>
ProxyDumpBVH::MergeDisjointTreelets(int dirIdx, int maxTreeletBytes,
                                    const TraversalGraph &graph) {
    unordered_map<uint32_t, IntermediateTreeletInfo> treelets;
    for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
        uint32_t curTreelet = treeletAllocations[dirIdx][nodeIdx];
        IntermediateTreeletInfo &treelet = treelets[curTreelet];
        treelet.proxies = ProxyUnion(treelet.proxies, nodeProxies[nodeIdx]);
        treelet.unsharedProxies.insert(treelet.unsharedProxies.end(),
                                       nodeUnsharedProxies[nodeIdx].begin(),
                                       nodeUnsharedProxies[nodeIdx].end());
        treelet.dirIdx = dirIdx;
        treelet.nodes.push_back(nodeIdx);
        // Don't want to count unshared proxies as raw node sizes, will double
        // count
        treelet.noProxySize +=
            nodeSizes[nodeIdx] - nodeUnsharedProxySizes[nodeIdx];
        treelet.unsharedProxySize += nodeUnsharedProxySizes[nodeIdx];

        auto outgoingBounds = graph.outgoing[nodeIdx];
        for (uint64_t edgeIdx = 0; edgeIdx < outgoingBounds.second; edgeIdx++) {
            Edge *edge = outgoingBounds.first + edgeIdx;
            uint32_t dstTreelet = treeletAllocations[dirIdx][edge->dst];
            if (curTreelet != dstTreelet) {
                IntermediateTreeletInfo &dstTreeletInfo = treelets[dstTreelet];
                dstTreeletInfo.totalProb += edge->weight;
            }
        }
    }

    for (auto &kv : treelets) {
        kv.second.proxySize = 0;
        for (const ProxyBVH *proxy : *(kv.second.proxies)) {
            kv.second.proxySize += proxy->Size();
        }
    }

    IntermediateTreeletInfo &rootTreelet =
        treelets.at(treeletAllocations[dirIdx][0]);
    rootTreelet.totalProb += 1.0;

    struct TreeletSortKey {
        uint32_t treeletID;
        uint64_t treeletSize;

        TreeletSortKey(uint32_t treeletID, uint64_t treeletSize)
            : treeletID(treeletID), treeletSize(treeletSize) {}
    };

    struct TreeletCmp {
        bool operator()(const TreeletSortKey &a,
                        const TreeletSortKey &b) const {
            if (a.treeletSize < b.treeletSize) {
                return true;
            }

            if (a.treeletSize > b.treeletSize) {
                return false;
            }

            return a.treeletID < b.treeletID;
        }
    };

    map<TreeletSortKey, IntermediateTreeletInfo, TreeletCmp> sortedTreelets;
    for (auto &kv : treelets) {
        CHECK_NE(kv.first, 0);
        CHECK_LE(kv.second.noProxySize + kv.second.proxySize +
                     kv.second.unsharedProxySize,
                 maxTreeletBytes);
        sortedTreelets.emplace(
            piecewise_construct,
            forward_as_tuple(kv.first, kv.second.noProxySize +
                                           kv.second.proxySize +
                                           kv.second.unsharedProxySize),
            forward_as_tuple(move(kv.second)));
    }

    // Merge treelets together
    unordered_map<uint32_t, IntermediateTreeletInfo> mergedTreelets;

    auto iter = sortedTreelets.begin();
    while (iter != sortedTreelets.end()) {
        IntermediateTreeletInfo &info = iter->second;

        auto candidateIter = next(iter);
        while (candidateIter != sortedTreelets.end()) {
            auto nextCandidateIter = next(candidateIter);
            IntermediateTreeletInfo &candidateInfo = candidateIter->second;

            uint64_t noProxySize = info.noProxySize + candidateInfo.noProxySize;
            if (noProxySize > maxTreeletBytes) {
                candidateIter = nextCandidateIter;
                continue;
            }

            ProxySetPtr mergedProxies =
                ProxyUnion(info.proxies, candidateInfo.proxies);
            uint64_t mergedProxySize = GetProxyBytes(mergedProxies);
            uint64_t mergedUnsharedProxySize =
                info.unsharedProxySize + candidateInfo.unsharedProxySize;

            uint64_t totalSize =
                noProxySize + mergedProxySize + mergedUnsharedProxySize;
            if (totalSize <= maxTreeletBytes) {
                if (info.nodes.front() < candidateInfo.nodes.front()) {
                    info.nodes.splice(info.nodes.end(),
                                      move(candidateInfo.nodes));
                } else {
                    candidateInfo.nodes.splice(candidateInfo.nodes.end(),
                                               move(info.nodes));
                    info.nodes = move(candidateInfo.nodes);
                }
                info.proxies = mergedProxies;
                info.unsharedProxies.splice(
                    info.unsharedProxies.end(),
                    move(candidateInfo.unsharedProxies));
                info.noProxySize = noProxySize;
                info.proxySize = mergedProxySize;
                info.unsharedProxySize = mergedUnsharedProxySize;
                info.totalProb += candidateInfo.totalProb;
                sortedTreelets.erase(candidateIter);
            }

            // No point searching further
            if (totalSize >= maxTreeletBytes - SizeEstimates::nodeSize) {
                break;
            }

            candidateIter = nextCandidateIter;
        }

        auto nextIter = next(iter);

        mergedTreelets.emplace(iter->first.treeletID, move(info));

        sortedTreelets.erase(iter);

        iter = nextIter;
    }

    return mergedTreelets;
}

void ProxyDumpBVH::OrderTreeletNodesDepthFirst(int numDirs,
                                               vector<TreeletInfo> &treelets) {
    // Reorder nodes to be depth first (left then right) for serialization
    for (uint32_t treeletID = 0; treeletID < treelets.size(); treeletID++) {
        TreeletInfo &treelet = treelets[treeletID];
        for (int nodeIdx : treelet.nodes) {
            treeletAllocations[treelet.dirIdx][nodeIdx] = treeletID;
        }
        treelet.nodes.clear();
    }

    for (int dirIdx = 0; dirIdx < numDirs; dirIdx++) {
        stack<uint64_t> depthFirst;
        depthFirst.push(0);

        while (!depthFirst.empty()) {
            uint64_t start = depthFirst.top();
            depthFirst.pop();

            uint32_t treeletID = treeletAllocations[dirIdx][start];
            TreeletInfo &info = treelets[treeletID];

            stack<uint64_t> depthFirstInTreelet;
            depthFirstInTreelet.push(start);
            while (!depthFirstInTreelet.empty()) {
                uint64_t nodeIdx = depthFirstInTreelet.top();
                depthFirstInTreelet.pop();
                info.nodes.push_back(nodeIdx);
                const LinearBVHNode &node = nodes[nodeIdx];
                if (node.nPrimitives == 0) {
                    uint32_t rightTreeletID =
                        treeletAllocations[dirIdx][node.secondChildOffset];
                    if (rightTreeletID == treeletID) {
                        depthFirstInTreelet.push(node.secondChildOffset);
                    } else {
                        depthFirst.push(node.secondChildOffset);
                    }

                    uint32_t leftTreeletID =
                        treeletAllocations[dirIdx][nodeIdx + 1];
                    if (leftTreeletID == treeletID) {
                        depthFirstInTreelet.push(nodeIdx + 1);
                    } else {
                        depthFirst.push(nodeIdx + 1);
                    }
                }
            }
        }
    }
}

vector<ProxyDumpBVH::TreeletInfo> ProxyDumpBVH::AllocateUnspecializedTreelets(
    int maxTreeletBytes) {
    TraversalGraph graph;
    graph.outgoing.resize(nodeCount);
    graph.incomingProb.resize(nodeCount);

    if (partitionAlgo == PartitionAlgorithm::MergedGraph) {
        vector<unordered_map<uint64_t, float>> mergedEdges;
        mergedEdges.resize(nodeCount);
        for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
            Vector3f dir = ComputeRayDir(dirIdx);
            TraversalGraph g = CreateTraversalGraph(dir, 0);
            if (dirIdx == 0) {
                graph.depthFirst = move(g.depthFirst);
            }
            for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
                graph.incomingProb[nodeIdx] += g.incomingProb[nodeIdx];

                const auto bounds = g.outgoing[nodeIdx];
                for (Edge *edge = bounds.first;
                     edge < bounds.first + bounds.second; edge++) {
                    mergedEdges[edge->src][edge->dst] += edge->weight;
                    mergedEdges[edge->dst][edge->src] += edge->weight;
                }
            }
        }

        uint64_t totalEdges = 0;
        for (auto &outgoing : mergedEdges) {
            totalEdges += outgoing.size();
        }
        graph.edges.reserve(totalEdges);
        for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
            auto &outgoing = mergedEdges[nodeIdx];
            size_t start = graph.edges.size();
            for (auto &kv : outgoing) {
                graph.edges.emplace_back(nodeIdx, kv.first, kv.second);
            }
            graph.outgoing[nodeIdx] =
                make_pair(graph.edges.data() + start, outgoing.size());
        }
    }

    treeletAllocations[0] = ComputeTreelets(graph, maxTreeletBytes);
    auto intermediateTreelets =
        MergeDisjointTreelets(0, maxTreeletBytes, graph);

    vector<TreeletInfo> finalTreelets;
    for (auto iter = intermediateTreelets.begin();
         iter != intermediateTreelets.end(); iter++) {
        IntermediateTreeletInfo &info = iter->second;
        if (info.nodes.front() == 0) {
            finalTreelets.emplace_back(move(info));
            intermediateTreelets.erase(iter);
            break;
        }
    }

    CHECK_EQ(finalTreelets.size(), 1);

    for (auto iter = intermediateTreelets.begin();
         iter != intermediateTreelets.end(); iter++) {
        finalTreelets.emplace_back(move(iter->second));
    }

    OrderTreeletNodesDepthFirst(1, finalTreelets);

    // Check that every node is in one treelet exactly once
    vector<uint64_t> nodeCheck(nodeCount);
    for (const TreeletInfo &treelet : finalTreelets) {
        for (uint64_t nodeIdx : treelet.nodes) {
            nodeCheck[nodeIdx] += 1;
        }
    }

    for (uint64_t count : nodeCheck) {
        CHECK_EQ(count, 1);
    }

    cout << "Final treelets: " << finalTreelets.size() << endl;
    uint64_t totalBytes = 0;
    for (int i = 0; i < finalTreelets.size(); i++) {
        const auto &treelet = finalTreelets[i];
        cout << "Treelet " << i << ": "
             << treelet.noProxySize + treelet.proxySize << endl;
        totalBytes += treelet.noProxySize + treelet.proxySize;
    }
    cout << "Total bytes: " << totalBytes << endl;

    return finalTreelets;
}

vector<ProxyDumpBVH::TreeletInfo> ProxyDumpBVH::AllocateDirectionalTreelets(
    int maxTreeletBytes) {
    array<unordered_map<uint32_t, IntermediateTreeletInfo>, 8>
        intermediateTreelets;

    for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
        Vector3f dir = ComputeRayDir(dirIdx);
        TraversalGraph graph = CreateTraversalGraph(dir, 0);

        // rayCounts[i].resize(nodeCount);
        //// Init rayCounts so unordered_map isn't modified during intersection
        // for (uint64_t srcIdx = 0; srcIdx < nodeCount; srcIdx++) {
        //     auto outgoing = graph.outgoing[srcIdx];
        //     for (const Edge *outgoingEdge = outgoing.first;
        //          outgoingEdge < outgoing.first + outgoing.second;
        //          outgoingEdge++) {
        //         uint64_t dstIdx = outgoingEdge->dst;
        //         auto res = rayCounts[i][srcIdx].emplace(dstIdx, 0);
        //         CHECK_EQ(res.second, true);
        //     }
        // }

        treeletAllocations[dirIdx] = ComputeTreelets(graph, maxTreeletBytes);
        intermediateTreelets[dirIdx] =
            MergeDisjointTreelets(dirIdx, maxTreeletBytes, graph);
    }

    vector<TreeletInfo> finalTreelets;
    // Assign root treelets to IDs 0 to 8
    for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
        // Search for treelet that holds node 0
        for (auto iter = intermediateTreelets[dirIdx].begin();
             iter != intermediateTreelets[dirIdx].end(); iter++) {
            IntermediateTreeletInfo &info = iter->second;
            if (info.nodes.front() == 0) {
                finalTreelets.push_back(move(info));
                intermediateTreelets[dirIdx].erase(iter);
                break;
            }
        }
    }
    CHECK_EQ(finalTreelets.size(), 8);

    // Assign the rest contiguously
    for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
        for (auto &p : intermediateTreelets[dirIdx]) {
            IntermediateTreeletInfo &treelet = p.second;
            finalTreelets.emplace_back(move(treelet));
        }
    }

    OrderTreeletNodesDepthFirst(8, finalTreelets);

    // Check that every node is in one treelet exactly once
    array<vector<uint64_t>, 8> nodeCheck;
    for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
        nodeCheck[dirIdx] = vector<uint64_t>(nodeCount);
    }

    for (const TreeletInfo &treelet : finalTreelets) {
        for (uint64_t nodeIdx : treelet.nodes) {
            nodeCheck[treelet.dirIdx][nodeIdx] += 1;
        }
    }

    for (int dirIdx = 0; dirIdx < 8; dirIdx++) {
        for (uint64_t count : nodeCheck[dirIdx]) {
            CHECK_EQ(count, 1);
        }
    }

    return finalTreelets;
}

vector<ProxyDumpBVH::TreeletInfo> ProxyDumpBVH::AllocateTreelets(
    int maxTreeletBytes) {
    if (partitionAlgo == PartitionAlgorithm::MergedGraph ||
        partitionAlgo == PartitionAlgorithm::Nvidia) {
        return AllocateUnspecializedTreelets(maxTreeletBytes);
    } else {
        return AllocateDirectionalTreelets(maxTreeletBytes);
    }
}

ProxyDumpBVH::IntermediateTraversalGraph
ProxyDumpBVH::CreateTraversalGraphSendCheck(const Vector3f &rayDir,
                                            int depthReduction) const {
    (void)depthReduction;
    IntermediateTraversalGraph g;
    g.depthFirst.reserve(nodeCount);
    g.outgoing.resize(nodeCount);
    g.incomingProb.resize(nodeCount);

    bool dirIsNeg[3] = {rayDir.x < 0, rayDir.y < 0, rayDir.z < 0};

    auto addEdge = [this, &g](auto src, auto dst, auto prob) {
        g.edges.emplace_back(src, dst, prob);

        if (g.outgoing[src].second == 0) {  // No outgoing yet
            g.outgoing[src].first = g.edges.size() - 1;
        }
        g.outgoing[src].second++;

        g.incomingProb[dst] += prob;
    };

    vector<uint64_t> traversalStack{0};
    traversalStack.reserve(64);

    g.incomingProb[0] = 1.0;
    while (traversalStack.size() > 0) {
        uint64_t curIdx = traversalStack.back();
        traversalStack.pop_back();
        g.depthFirst.push_back(curIdx);

        LinearBVHNode *node = &nodes[curIdx];
        float curProb = g.incomingProb[curIdx];
        CHECK_GT(curProb, 0.0);
        CHECK_LE(curProb, 1.0001);  // FP error (should be 1.0)

        uint64_t nextHit = 0, nextMiss = 0;
        if (traversalStack.size() > 0) {
            nextMiss = traversalStack.back();
        }

        if (node->nPrimitives == 0) {
            if (dirIsNeg[node->axis]) {
                traversalStack.push_back(curIdx + 1);
                traversalStack.push_back(node->secondChildOffset);
            } else {
                traversalStack.push_back(node->secondChildOffset);
                traversalStack.push_back(curIdx + 1);
            }

            nextHit = traversalStack.back();
            LinearBVHNode *nextHitNode = &nodes[nextHit];

            if (nextMiss == 0) {
                // Guaranteed move down in the BVH
                CHECK_GT(curProb, 0.99);  // FP error (should be 1.0)
                addEdge(curIdx, nextHit, curProb);
            } else {
                LinearBVHNode *nextMissNode = &nodes[nextMiss];

                float curSA = node->bounds.SurfaceArea();
                float nextSA = nextHitNode->bounds.SurfaceArea();

                float condHitProb = nextSA / curSA;
                CHECK_LE(condHitProb, 1.0);
                float condMissProb = 1.0 - condHitProb;

                float hitPathProb = curProb * condHitProb;
                float missPathProb = curProb * condMissProb;

                addEdge(curIdx, nextHit, hitPathProb);
                addEdge(curIdx, nextMiss, missPathProb);
            }
        } else if (nextMiss != 0) {
            // If this is a leaf node with a non copyable instance at the end
            // of the primitive list, the edge from curIdx to nextMiss should
            // not exist, because in reality there should be an edge from
            // curIdx to the instance, and from the instance to nextMiss.
            // nextMiss should still receive the incomingProb since the instance
            // edges are never represented in the graph.
            bool skipEdge = false;
            auto &lastPrim =
                primitives[node->primitivesOffset + node->nPrimitives - 1];
            if (lastPrim->GetType() == PrimitiveType::Transformed) {
                shared_ptr<TransformedPrimitive> tp =
                    dynamic_pointer_cast<TransformedPrimitive>(lastPrim);
                shared_ptr<ProxyBVH> proxy =
                    dynamic_pointer_cast<ProxyBVH>(tp->GetPrimitive());
                CHECK_NOTNULL(proxy.get());
                if (largeProxies.count(proxy.get())) {
                    skipEdge = true;
                }
            }

            // Leaf node, guaranteed move up in the BVH
            if (skipEdge) {
                g.incomingProb[nextMiss] += curProb;
            } else {
                addEdge(curIdx, nextMiss, curProb);
            }
        } else {
            // Termination point for all traversal paths
            CHECK_EQ(traversalStack.size(), 0);
            CHECK_GT(curProb, 0.99);
        }
    }

    return g;
}

ProxyDumpBVH::IntermediateTraversalGraph
ProxyDumpBVH::CreateTraversalGraphCheckSend(const Vector3f &rayDir,
                                            int depthReduction) const {
    (void)depthReduction;
    IntermediateTraversalGraph g;
    g.depthFirst.reserve(nodeCount);
    g.outgoing.resize(nodeCount);
    g.incomingProb.resize(nodeCount);

    bool dirIsNeg[3] = {rayDir.x < 0, rayDir.y < 0, rayDir.z < 0};

    // FIXME this should just be a graph method
    auto addEdge = [this, &g](auto src, auto dst, auto prob) {
        g.edges.emplace_back(src, dst, prob);

        if (g.outgoing[src].second == 0) {  // No outgoing yet
            g.outgoing[src].first = g.edges.size() - 1;
        }
        g.outgoing[src].second++;

        g.incomingProb[dst] += prob;
    };

    vector<uint64_t> traversalStack{0};
    traversalStack.reserve(64);

    g.incomingProb[0] = 1.0;
    while (traversalStack.size() > 0) {
        uint64_t curIdx = traversalStack.back();
        traversalStack.pop_back();
        g.depthFirst.push_back(curIdx);

        LinearBVHNode *node = &nodes[curIdx];
        float curProb = g.incomingProb[curIdx];
        CHECK_GE(curProb, 0.0);
        CHECK_LE(curProb, 1.0001);  // FP error (should be 1.0)

        if (node->nPrimitives == 0) {
            if (dirIsNeg[node->axis]) {
                traversalStack.push_back(curIdx + 1);
                traversalStack.push_back(node->secondChildOffset);
            } else {
                traversalStack.push_back(node->secondChildOffset);
                traversalStack.push_back(curIdx + 1);
            }
        }

        // refer to SendCheck for explanation
        bool skipEdge = false;
        if (node->nPrimitives > 0) {
            auto &lastPrim =
                primitives[node->primitivesOffset + node->nPrimitives - 1];
            if (lastPrim->GetType() == PrimitiveType::Transformed) {
                shared_ptr<TransformedPrimitive> tp =
                    dynamic_pointer_cast<TransformedPrimitive>(lastPrim);
                shared_ptr<ProxyBVH> proxy =
                    dynamic_pointer_cast<ProxyBVH>(tp->GetPrimitive());
                if (largeProxies.count(proxy.get())) {
                    skipEdge = true;
                }
            }
        }

        float runningProb = 1.0;
        for (uint64_t i = traversalStack.size(); i-- > 0; i--) {
            uint64_t nextNode = traversalStack[i];
            LinearBVHNode *nextHitNode = &nodes[nextNode];
            LinearBVHNode *parentHitNode = &nodes[nodeParents[nextNode]];

            // FIXME ask Pat about this
            float nextSA = nextHitNode->bounds.SurfaceArea();
            float parentSA = parentHitNode->bounds.SurfaceArea();

            float condHitProb = nextSA / parentSA;
            CHECK_LE(condHitProb, 1.0);
            float pathProb = curProb * runningProb * condHitProb;

            if (skipEdge) {
                g.incomingProb[nextNode] += pathProb;
            } else {
                addEdge(curIdx, nextNode, pathProb);
            }
            // runningProb can become 0 here if condHitProb == 1
            // could break, but then edges don't get added and Intersect
            // may crash if it turns out that edge gets taken
            runningProb *= 1.0 - condHitProb;
        }
        CHECK_LE(runningProb, 1.0);
        CHECK_GE(runningProb, 0.0);
    }

    return g;
}

ProxyDumpBVH::TraversalGraph ProxyDumpBVH::CreateTraversalGraph(
    const Vector3f &rayDir, int depthReduction) const {
    cout << "Starting graph gen\n";
    IntermediateTraversalGraph intermediate;

    // FIXME fix probabilities here on up edges

    switch (traversalAlgo) {
    case TraversalAlgorithm::SendCheck:
        intermediate = CreateTraversalGraphSendCheck(rayDir, depthReduction);
        break;
    case TraversalAlgorithm::CheckSend:
        intermediate = CreateTraversalGraphCheckSend(rayDir, depthReduction);
        break;
    }
    cout << "Intermediate finished\n";

    // Remake graph with contiguous vectors
    TraversalGraph graph;
    auto edgeIter = intermediate.edges.begin();
    while (edgeIter != intermediate.edges.end()) {
        graph.edges.push_back(*edgeIter);
        edgeIter++;
        intermediate.edges.pop_front();
    }

    graph.depthFirst = move(intermediate.depthFirst);
    graph.incomingProb = move(intermediate.incomingProb);

    auto adjacencyIter = intermediate.outgoing.begin();
    while (adjacencyIter != intermediate.outgoing.end()) {
        uint64_t idx = adjacencyIter->first;
        uint64_t weight = adjacencyIter->second;
        graph.outgoing.emplace_back(&graph.edges[idx], weight);
        adjacencyIter++;
        intermediate.outgoing.pop_front();
    }

    printf("Graph gen complete: %lu verts %lu edges\n", graph.depthFirst.size(),
           graph.edges.size());

    return graph;
}

vector<uint32_t> ProxyDumpBVH::ComputeTreeletsTopological(
    const TraversalGraph &graph, uint64_t maxTreeletBytes) const {
    struct OutEdge {
        float weight;
        uint64_t dst;

        OutEdge(const Edge &edge) : weight(edge.weight), dst(edge.dst) {}
    };

    struct EdgeCmp {
        bool operator()(const OutEdge &a, const OutEdge &b) const {
            if (a.weight > b.weight) {
                return true;
            }

            if (a.weight < b.weight) {
                return false;
            }

            return a.dst < b.dst;
        }
    };

    vector<uint32_t> assignment(nodeCount);
    list<uint64_t> depthFirst;
    vector<decltype(depthFirst)::iterator> sortLocs(nodeCount);
    for (uint64_t nodeIdx : graph.depthFirst) {
        depthFirst.push_back(nodeIdx);
        sortLocs[nodeIdx] = --depthFirst.end();
    }

    uint32_t curTreelet = 1;
    while (!depthFirst.empty()) {
        uint64_t curNode = depthFirst.front();
        depthFirst.pop_front();
        assignment[curNode] = curTreelet;

        set<OutEdge, EdgeCmp> cut;
        unordered_map<uint64_t, decltype(cut)::iterator> uniqueLookup;
        ProxySetPtr includedProxies = nodeProxies[curNode];

        // Accounts for size of this node + the size of new instances that would
        // be pulled in
        auto getAdditionalSize = [this, &includedProxies](uint64_t nodeIdx) {
            const LinearBVHNode &node = nodes[nodeIdx];

            uint64_t totalSize = nodeSizes[nodeIdx];

            for (const ProxyBVH *proxy : *(nodeProxies[nodeIdx])) {
                if (!includedProxies->count(proxy)) {
                    totalSize += proxy->Size();
                }
            }

            return totalSize;
        };

        uint64_t rootSize = getAdditionalSize(curNode);
        // If this is false the node is too big to fit in any treelet
        CHECK_LE(rootSize, maxTreeletBytes);

        uint64_t remainingBytes = maxTreeletBytes - rootSize;

        while (remainingBytes >= sizeof(CloudBVH::TreeletNode)) {
            auto outgoingBounds = graph.outgoing[curNode];
            for (uint64_t i = 0; i < outgoingBounds.second; i++) {
                const Edge *edge = outgoingBounds.first + i;

                uint64_t nodeSize = getAdditionalSize(edge->dst);
                if (nodeSize > remainingBytes) continue;

                auto preexisting = uniqueLookup.find(edge->dst);
                if (preexisting == uniqueLookup.end()) {
                    auto res = cut.emplace(*edge);
                    CHECK_EQ(res.second, true);
                    uniqueLookup.emplace(edge->dst, res.first);
                } else {
                    auto &iter = preexisting->second;
                    OutEdge update = *iter;
                    CHECK_EQ(update.dst, edge->dst);
                    update.weight += edge->weight;

                    cut.erase(iter);
                    auto res = cut.insert(update);
                    CHECK_EQ(res.second, true);
                    iter = res.first;
                }
            }

            uint64_t usedBytes = 0;
            auto bestEdge = cut.end();

            auto edge = cut.begin();
            while (edge != cut.end()) {
                auto nextEdge = next(edge);
                uint64_t dst = edge->dst;
                uint64_t curBytes = getAdditionalSize(dst);
                float curWeight = edge->weight;

                // This node already belongs to a treelet
                if (assignment[dst] != 0 || curBytes > remainingBytes) {
                    cut.erase(edge);
                    auto eraseRes = uniqueLookup.erase(dst);
                    CHECK_EQ(eraseRes, 1);
                } else {
                    usedBytes = curBytes;
                    bestEdge = edge;
                    break;
                }

                edge = nextEdge;
            }

            // Treelet full
            if (bestEdge == cut.end()) {
                break;
            }

            cut.erase(bestEdge);
            auto eraseRes = uniqueLookup.erase(bestEdge->dst);
            CHECK_EQ(eraseRes, 1);

            curNode = bestEdge->dst;

            depthFirst.erase(sortLocs[curNode]);
            assignment[curNode] = curTreelet;
            remainingBytes -= usedBytes;
            includedProxies = ProxyUnion(includedProxies, nodeProxies[curNode]);
        }

        curTreelet++;
    }

    return assignment;
}

vector<uint32_t> ProxyDumpBVH::ComputeTreelets(const TraversalGraph &graph,
                                               uint64_t maxTreeletBytes) const {
    vector<uint32_t> assignment;
    switch (partitionAlgo) {
    case PartitionAlgorithm::Topological:
        assignment = ComputeTreeletsTopological(graph, maxTreeletBytes);
        break;
    case PartitionAlgorithm::MergedGraph:
        assignment = ComputeTreeletsTopological(graph, maxTreeletBytes);
        break;
    case PartitionAlgorithm::Nvidia:
        assignment = ComputeTreeletsNvidia(maxTreeletBytes);
        break;
    }

    uint64_t totalBytesStats = 0;
    map<uint32_t, uint64_t> sizes;
    unordered_map<uint32_t, ProxySetPtr> proxiesTracker;
    for (uint64_t nodeIdx = 0; nodeIdx < nodeCount; nodeIdx++) {
        uint32_t treelet = assignment[nodeIdx];
        CHECK_NE(treelet, 0);

        proxiesTracker[treelet] =
            ProxyUnion(proxiesTracker[treelet], nodeProxies[nodeIdx]);

        uint64_t bytes = nodeSizes[nodeIdx];

        sizes[treelet] += bytes;
        totalBytesStats += bytes;
    }

    for (auto &kv : proxiesTracker) {
        uint32_t treelet = kv.first;
        ProxySetPtr proxies = kv.second;
        for (const ProxyBVH *proxy : *proxies) {
            sizes[treelet] += proxy->Size();
            totalBytesStats += proxy->Size();
        }
    }

    printf("Generated %lu treelets: %lu total bytes from %ld nodes\n",
           sizes.size(), totalBytesStats, nodeCount);

    for (auto &sz : sizes) {
        CHECK_LE(sz.second, maxTreeletBytes);
        printf("Treelet %u: %lu bytes\n", sz.first, sz.second);
    }

    return assignment;
}

struct NvidiaCut {
    uint64_t nodeIdx;
    uint64_t additionalNodeSize;
    uint64_t additionalSubtreeSize;
    NvidiaCut(uint64_t n, uint64_t nodeBytes, uint64_t subtreeBytes)
        : nodeIdx(n),
          additionalNodeSize(nodeBytes),
          additionalSubtreeSize(subtreeBytes) {}
};

vector<uint32_t> ProxyDumpBVH::ComputeTreeletsNvidia(
    const uint64_t maxTreeletBytes) const {
    vector<uint32_t> labels(nodeCount);

    /* pass one */
    vector<float> best_costs(nodeCount, 0);

    for (uint64_t root_index = nodeCount; root_index-- > 0;) {
        const LinearBVHNode &root_node = nodes[root_index];

        vector<NvidiaCut> cut;
        cut.emplace_back(
            root_index, nodeSizes[root_index] + nodeProxySizes[root_index],
            subtreeSizes[root_index] + subtreeProxySizes[root_index]);
        best_costs[root_index] = std::numeric_limits<float>::max();

        unordered_set<const ProxyBVH *> included_proxies;
        uint64_t remaining_size = maxTreeletBytes;

        while (true) {
            auto best_node_iter = cut.end();
            float best_score = std::numeric_limits<float>::lowest();

            for (auto iter = cut.begin(); iter != cut.end(); iter++) {
                auto n = iter->nodeIdx;
                if (iter->additionalNodeSize > remaining_size) continue;

                float gain = nodeBounds[n];

                uint64_t price =
                    min(iter->additionalSubtreeSize, remaining_size);
                float score = gain / price;
                if (score > best_score) {
                    best_node_iter = iter;
                    best_score = score;
                }
            }

            if (best_node_iter == cut.end()) break;
            uint64_t best_node_index = best_node_iter->nodeIdx;
            remaining_size -= best_node_iter->additionalNodeSize;

            cut.erase(best_node_iter);

            auto &best_node_proxies = *(nodeProxies[best_node_index]);

            for (const ProxyBVH *proxy : best_node_proxies) {
                auto p = included_proxies.emplace(proxy);
                if (!p.second) continue;

                for (auto &cut_elem : cut) {
                    if (nodeProxies[cut_elem.nodeIdx]->count(proxy)) {
                        cut_elem.additionalNodeSize -= proxy->Size();
                    }

                    if (subtreeProxies[cut_elem.nodeIdx]->count(proxy)) {
                        cut_elem.additionalSubtreeSize -= proxy->Size();
                    }
                }
            }

            const LinearBVHNode &best_node = nodes[best_node_index];

            if (best_node.nPrimitives == 0) {
                uint64_t leftNodeAdditionalSize =
                    nodeSizes[best_node_index + 1] +
                    nodeProxySizes[best_node_index + 1];
                uint64_t leftSubtreeAdditionalSize =
                    subtreeSizes[best_node_index + 1] +
                    subtreeProxySizes[best_node_index + 1];

                uint64_t rightNodeAdditionalSize =
                    nodeSizes[best_node.secondChildOffset] +
                    nodeProxySizes[best_node.secondChildOffset];
                uint64_t rightSubtreeAdditionalSize =
                    subtreeSizes[best_node.secondChildOffset] +
                    subtreeProxySizes[best_node.secondChildOffset];

                for (const ProxyBVH *proxy : included_proxies) {
                    if (nodeProxies[best_node_index + 1]->count(proxy)) {
                        leftNodeAdditionalSize -= proxy->Size();
                    }

                    if (subtreeProxies[best_node_index + 1]->count(proxy)) {
                        leftSubtreeAdditionalSize -= proxy->Size();
                    }

                    if (nodeProxies[best_node.secondChildOffset]->count(
                            proxy)) {
                        rightNodeAdditionalSize -= proxy->Size();
                    }

                    if (subtreeProxies[best_node.secondChildOffset]->count(
                            proxy)) {
                        rightSubtreeAdditionalSize -= proxy->Size();
                    }
                }

                cut.emplace_back(best_node_index + 1, leftNodeAdditionalSize,
                                 leftSubtreeAdditionalSize);
                cut.emplace_back(best_node.secondChildOffset,
                                 rightNodeAdditionalSize,
                                 rightSubtreeAdditionalSize);
            }

            float this_cost = nodeBounds[root_index];
            for (const auto &cut_elem : cut) {
                this_cost += best_costs[cut_elem.nodeIdx];
            }
            best_costs[root_index] =
                std::min(best_costs[root_index], this_cost);
        }
    }

    auto float_equals = [](const float a, const float b) {
        return fabs(a - b) < 1e-4;
    };

    uint32_t current_treelet = 0;

    std::stack<uint64_t> q;
    q.push(0);

    uint64_t node_count = 0;

    while (not q.empty()) {
        const uint64_t root_index = q.top();

        q.pop();

        current_treelet++;

        const LinearBVHNode &root_node = nodes[root_index];
        vector<NvidiaCut> cut;
        cut.emplace_back(
            root_index, nodeSizes[root_index] + nodeProxySizes[root_index],
            subtreeSizes[root_index] + subtreeProxySizes[root_index]);

        uint64_t remaining_size = maxTreeletBytes;
        const float best_cost = best_costs[root_index];

        unordered_set<const ProxyBVH *> included_proxies;

        while (true) {
            auto best_node_iter = cut.end();
            float best_score = std::numeric_limits<float>::lowest();

            for (auto iter = cut.begin(); iter != cut.end(); iter++) {
                auto n = iter->nodeIdx;
                if (iter->additionalNodeSize > remaining_size) continue;

                float gain = nodeBounds[n];

                uint64_t price =
                    min(iter->additionalSubtreeSize, remaining_size);
                float score = gain / price;
                if (score > best_score) {
                    best_node_iter = iter;
                    best_score = score;
                }
            }

            if (best_node_iter == cut.end()) break;

            remaining_size -= best_node_iter->additionalNodeSize;
            uint64_t best_node_index = best_node_iter->nodeIdx;

            cut.erase(best_node_iter);

            auto &best_node_proxies = *(nodeProxies[best_node_index]);
            for (const ProxyBVH *proxy : best_node_proxies) {
                auto p = included_proxies.emplace(proxy);
                if (!p.second) continue;

                for (auto &cut_elem : cut) {
                    if (nodeProxies[cut_elem.nodeIdx]->count(proxy)) {
                        cut_elem.additionalNodeSize -= proxy->Size();
                    }

                    if (subtreeProxies[cut_elem.nodeIdx]->count(proxy)) {
                        cut_elem.additionalSubtreeSize -= proxy->Size();
                    }
                }
            }

            const LinearBVHNode &best_node = nodes[best_node_index];

            if (best_node.nPrimitives == 0) {
                uint64_t leftNodeAdditionalSize =
                    nodeSizes[best_node_index + 1] +
                    nodeProxySizes[best_node_index + 1];
                uint64_t leftSubtreeAdditionalSize =
                    subtreeSizes[best_node_index + 1] +
                    subtreeProxySizes[best_node_index + 1];

                uint64_t rightNodeAdditionalSize =
                    nodeSizes[best_node.secondChildOffset] +
                    nodeProxySizes[best_node.secondChildOffset];
                uint64_t rightSubtreeAdditionalSize =
                    subtreeSizes[best_node.secondChildOffset] +
                    subtreeProxySizes[best_node.secondChildOffset];

                for (const ProxyBVH *proxy : included_proxies) {
                    if (nodeProxies[best_node_index + 1]->count(proxy)) {
                        leftNodeAdditionalSize -= proxy->Size();
                    }

                    if (subtreeProxies[best_node_index + 1]->count(proxy)) {
                        leftSubtreeAdditionalSize -= proxy->Size();
                    }

                    if (nodeProxies[best_node.secondChildOffset]->count(
                            proxy)) {
                        rightNodeAdditionalSize -= proxy->Size();
                    }

                    if (subtreeProxies[best_node.secondChildOffset]->count(
                            proxy)) {
                        rightSubtreeAdditionalSize -= proxy->Size();
                    }
                }

                cut.emplace_back(best_node_index + 1, leftNodeAdditionalSize,
                                 leftSubtreeAdditionalSize);
                cut.emplace_back(best_node.secondChildOffset,
                                 rightNodeAdditionalSize,
                                 rightSubtreeAdditionalSize);
            }

            labels[best_node_index] = current_treelet;

            float this_cost = nodeBounds[root_index];
            for (const auto &cut_elem : cut) {
                this_cost += best_costs[cut_elem.nodeIdx];
            }

            if (float_equals(this_cost, best_cost)) {
                break;
            }
        }

        for (const auto &cut_elem : cut) {
            q.push(cut_elem.nodeIdx);
        }
    }

    return labels;
}

void ProxyDumpBVH::DumpSanityCheck(
    const vector<unordered_map<uint64_t, uint32_t>> &treeletNodeLocations)
    const {
    enum CHILD { LEFT = 0, RIGHT = 1 };

    // Sanity Check for deserializing
    for (uint32_t treeletID = 0; treeletID < allTreelets.size(); treeletID++) {
        const TreeletInfo &treelet = allTreelets[treeletID];
        stack<tuple<uint64_t, uint32_t, CHILD>> q;
        uint32_t serializedLoc = 0;

        for (uint64_t nodeIdx : treelet.nodes) {
            const LinearBVHNode &node = nodes[nodeIdx];

            if (!q.empty()) {
                auto parent = q.top();
                q.pop();
                uint64_t parentIdx = get<0>(parent);
                uint32_t parentLoc = get<1>(parent);
                CHILD child = get<2>(parent);

                uint64_t realParent = nodeParents[nodeIdx];

                CHECK_EQ(parentIdx, realParent);
                const LinearBVHNode &parentNode = nodes[parentIdx];
                if (child == LEFT) {
                    CHECK_EQ(&node - &parentNode, 1);
                }
                if (child == RIGHT) {
                    CHECK_EQ(&node - &nodes[0], parentNode.secondChildOffset);
                }
            }

            if (node.nPrimitives == 0) {
                uint64_t leftNodeIdx = nodeIdx + 1;
                uint64_t rightNodeIdx = node.secondChildOffset;

                uint32_t leftTreelet =
                    treeletAllocations[treelet.dirIdx][leftNodeIdx];
                uint32_t rightTreelet =
                    treeletAllocations[treelet.dirIdx][rightNodeIdx];

                if (rightTreelet == treeletID) {
                    q.emplace(nodeIdx, serializedLoc, RIGHT);
                }

                if (leftTreelet == treeletID) {
                    q.emplace(nodeIdx, serializedLoc, LEFT);
                }
            }

            serializedLoc++;
        }
    }
}

void ProxyDumpBVH::DumpHeader() const {
    const string dir = _manager.getScenePath();
    ofstream header(dir + "/HEADER");
    Bounds3f root = nodes[0].bounds;
    header.write(reinterpret_cast<char *>(&root), sizeof(Bounds3f));

    uint64_t allTreeletsSize = 0;
    for (const TreeletInfo &treelet : allTreelets) {
        allTreeletsSize += treelet.noProxySize;
    }
    header.write(reinterpret_cast<const char *>(&allTreeletsSize),
                 sizeof(uint64_t));

    header.write(reinterpret_cast<const char *>(&nodeCount), sizeof(uint64_t));

    uint64_t numDeps = allProxies.size();
    header.write(reinterpret_cast<const char *>(&numDeps), sizeof(uint64_t));
    for (const ProxyBVH *proxy : allProxies) {
        string name = proxy->Name();
        uint64_t nameSize = name.size();
        header.write(reinterpret_cast<const char *>(&nameSize),
                     sizeof(uint64_t));
        header.write(name.c_str(), nameSize);
    }

    header.close();
}

shared_ptr<TriangleMesh> cutMesh(
    const uint32_t newMeshId, TriangleMesh *mesh, const vector<size_t> &triNums,
    unordered_map<size_t, pair<size_t, size_t>> &triNumRemap,
    function<int(const int)> faceRemap = [](const int a) { return a; });

vector<uint32_t> ProxyDumpBVH::DumpTreelets(bool root,
                                            bool inlineProxies) const {
    // Assign IDs to each treelet
    for (const TreeletInfo &treelet : allTreelets) {
        _manager.getNextId(ObjectType::Treelet, &treelet);
    }

    bool multiDir = false;
    for (const TreeletInfo &info : allTreelets) {
        if (info.dirIdx != 0) {
            multiDir = true;
            break;
        }
    }

    vector<unordered_map<uint64_t, uint32_t>> treeletNodeLocations(
        allTreelets.size());
    vector<unordered_map<const ProxyBVH *, uint32_t>> treeletProxyStarts(
        allTreelets.size());
    for (uint32_t treeletID = 0; treeletID < allTreelets.size(); treeletID++) {
        const TreeletInfo &treelet = allTreelets[treeletID];
        uint32_t listIdx = 0;
        for (uint64_t nodeIdx : treelet.nodes) {
            treeletNodeLocations[treeletID][nodeIdx] = listIdx;
            listIdx++;
        }

        uint32_t proxyIdx = treelet.nodes.size();
        for (const ProxyBVH *proxy : treelet.proxies) {
            treeletProxyStarts[treeletID].emplace(proxy, proxyIdx);
            proxyIdx += proxy->nodeCount();
        }
    }

    DumpSanityCheck(treeletNodeLocations);

    map<MaterialKey, MaterialKey> materialKeyRemap;

    auto duplicateTreelet = [&](unique_ptr<FileRecordReader> &reader,
                                unique_ptr<LiteRecordWriter> &writer,
                                const uint32_t oldTreeletId,
                                const uint32_t newTreeletId,
                                const vector<uint32_t> &mapping) {
        const auto numImgParts = reader->read<uint32_t>();
        CHECK_EQ(numImgParts, 0);
        writer->write(numImgParts);

        map<string, string> texRemap;
        map<uint32_t, uint32_t> ftexRemap, stexRemap;
        map<uint32_t, uint32_t> meshRemap;

        const auto numTexs = reader->read<uint32_t>();
        writer->write(numTexs);
        for (uint32_t i = 0; i < numTexs; i++) {
            const auto id = reader->read<uint32_t>();
            const auto len = reader->next_record_size();
            unique_ptr<char[]> storage{make_unique<char[]>(len)};
            reader->read(storage.get(), len);

            const auto newId = _manager.getNextId(ObjectType::Texture);
            writer->write(newId);
            writer->write(storage.get(), len);

            texRemap[_manager.getFileName(ObjectType::Texture, id)] =
                _manager.getFileName(ObjectType::Texture, newId);
        }

        const auto numStexs = reader->read<uint32_t>();
        writer->write(numStexs);
        for (uint32_t i = 0; i < numStexs; i++) {
            const auto id = reader->read<uint32_t>();
            const auto data = reader->read<string>();

            protobuf::SpectrumTexture stex_proto;
            stex_proto.ParseFromString(data);
            auto params = stex_proto.mutable_params();
            for (size_t j = 0; j < params->strings_size(); j++) {
                if (params->strings(j).name() == "filename") {
                    params->mutable_strings(j)->set_values(
                        0, texRemap.at(params->strings(j).values(0)));
                }
            }

            const auto newId = _manager.getNextId(ObjectType::SpectrumTexture);
            writer->write(newId);
            writer->write(protoutil::to_string(stex_proto));

            stexRemap[id] = newId;
        }

        const auto numFtexs = reader->read<uint32_t>();
        writer->write(numFtexs);
        for (uint32_t i = 0; i < numFtexs; i++) {
            const auto id = reader->read<uint32_t>();
            const auto data = reader->read<string>();

            protobuf::FloatTexture ftex_proto;
            ftex_proto.ParseFromString(data);
            auto params = ftex_proto.mutable_params();
            for (size_t j = 0; j < params->strings_size(); j++) {
                if (params->strings(j).name() == "filename") {
                    params->mutable_strings(j)->set_values(
                        0, texRemap.at(params->strings(j).values(0)));
                }
            }

            const auto newId = _manager.getNextId(ObjectType::FloatTexture);
            writer->write(newId);
            writer->write(protoutil::to_string(ftex_proto));

            ftexRemap[id] = newId;
        }

        const auto numMats = reader->read<uint32_t>();
        for (uint32_t i = 0; i < numFtexs; i++) {
            const auto id = reader->read<uint32_t>();
            const auto data = reader->read<string>();
            protobuf::Material mat_proto;
            mat_proto.ParseFromString(data);

            for (auto &kv : *mat_proto.mutable_spectrum_textures()) {
                kv.second = stexRemap[kv.second];
            }

            for (auto &kv : *mat_proto.mutable_float_textures()) {
                kv.second = ftexRemap[kv.second];
            }

            const auto newId = _manager.getNextId(ObjectType::Material);
            writer->write(newId);
            writer->write(protoutil::to_string(mat_proto));

            materialKeyRemap[{oldTreeletId, id}] = {newTreeletId, newId};
        }

        const auto numMeshes = reader->read<uint32_t>();
        writer->write(numMeshes);

        for (size_t i = 0; i < numMeshes; i++) {
            const auto meshId = reader->read<uint64_t>();
            const auto matKey = reader->read<MaterialKey>();
            const auto areaLightId = reader->read<uint32_t>();

            CHECK_EQ(areaLightId, 0);

            const size_t len = reader->next_record_size();
            unique_ptr<char[]> storage{make_unique<char[]>(len)};
            reader->read(storage.get(), len);

            const auto newId = _manager.getNextId(ObjectType::TriangleMesh);
            writer->write(static_cast<uint64_t>(newId));
            writer->write(materialKeyRemap[matKey]);
            writer->write(areaLightId);
            writer->write(storage.get(), len);

            meshRemap[meshId] = newId;
        }

        const auto nodeCount = reader->read<uint32_t>();
        const auto primCount = reader->read<uint32_t>();
        writer->write(nodeCount);
        writer->write(primCount);

        vector<CloudBVH::TreeletNode> nodes;
        nodes.resize(nodeCount);
        reader->read(reinterpret_cast<char *>(nodes.data()),
                     sizeof(CloudBVH::TreeletNode) * nodes.size());

        for (auto &node : nodes) {
            if (!node.is_leaf()) {
                node.child_treelet[0] = mapping[node.child_treelet[0]];
                node.child_treelet[1] = mapping[node.child_treelet[1]];
            }
        }

        writer->write(reinterpret_cast<char *>(nodes.data()),
                      sizeof(CloudBVH::TreeletNode) * nodes.size());

        serdes::cloudbvh::TransformedPrimitive serdesTransformed;
        serdes::cloudbvh::Triangle serdesTriangle;

        for (auto &node : nodes) {
            const uint32_t transformedCount = reader->read<uint32_t>();
            const uint32_t triangleCount = reader->read<uint32_t>();

            writer->write(transformedCount);
            writer->write(triangleCount);

            for (uint32_t i = 0; i < transformedCount; i++) {
                reader->read(&serdesTransformed);

                uint64_t rootRef = serdesTransformed.root_ref;
                uint32_t oldTreelet = (uint32_t)(rootRef >> 32);
                uint32_t newTreelet = mapping[oldTreelet];
                uint64_t newRootRef = newTreelet;
                newRootRef <<= 32;
                newRootRef |= (uint32_t)(rootRef);

                writer->write(serdesTransformed);
            }

            for (uint32_t i = 0; i < triangleCount; i++) {
                reader->read(&serdesTriangle);
                serdesTriangle.mesh_id = meshRemap[serdesTriangle.mesh_id];
                writer->write(serdesTriangle);
            }
        }
    };

    unordered_map<const ProxyBVH *, vector<uint32_t>> nonCopyableProxyRoots;

    if (inlineProxies) {
        for (const ProxyBVH *large : largeProxies) {
            auto readers = large->GetReaders();
            CHECK_GT(readers.size(), 0);

            // assign ids
            vector<uint32_t> id_remap;
            for (auto &reader : readers) {
                const uint32_t newId = _manager.getNextId(ObjectType::Treelet,
                                                          reader.second.get());
                id_remap.push_back(newId);
            }

            if (!multiDir) {
                nonCopyableProxyRoots[large].push_back(
                    _manager.getId(readers[0].second.get()));
            } else {
                for (int i = 0; i < 8; i++) {
                    nonCopyableProxyRoots[large].push_back(
                        _manager.getId(readers[i].second.get()));
                }
            }

            // Redump
            // FIXME if a large proxy references proxies that it expects
            // to inline this will be wrong
            for (auto it = readers.rbegin(); it != readers.rend(); it++) {
                const auto oldId = it->first;
                auto &reader = it->second;

                uint32_t newId = _manager.getId(reader.get());

                auto writer = make_unique<LiteRecordWriter>(
                    _manager.getFilePath(ObjectType::Treelet, newId));

                duplicateTreelet(reader, writer, oldId, newId, id_remap);
            }
        }
    }

    for (uint32_t treeletID = 0; treeletID < allTreelets.size(); treeletID++) {
        const TreeletInfo &treelet = allTreelets[treeletID];
        // Find which triangles / meshes are in treelet
        unordered_map<TriangleMesh *, vector<size_t>> trianglesInTreelet;
        for (uint64_t nodeIdx : treelet.nodes) {
            const LinearBVHNode &node = nodes[nodeIdx];
            for (int primIdx = 0; primIdx < node.nPrimitives; primIdx++) {
                auto &prim = primitives[node.primitivesOffset + primIdx];
                if (prim->GetType() == PrimitiveType::Geometric) {
                    shared_ptr<GeometricPrimitive> gp =
                        dynamic_pointer_cast<GeometricPrimitive>(prim);
                    const Shape *shape = gp->GetShape();
                    const Triangle *tri = dynamic_cast<const Triangle *>(shape);
                    CHECK_NOTNULL(tri);
                    TriangleMesh *mesh = tri->mesh.get();

                    uint64_t triNum = (tri->v - tri->mesh->vertexIndices) / 3;

                    CHECK_GE(triNum, 0);
                    trianglesInTreelet[mesh].push_back(triNum);
                }
            }
        }

        uint32_t numProxyMeshes = 0;
        if (inlineProxies) {
            for (const ProxyBVH *proxy : treelet.proxies) {
                auto readers = proxy->GetReaders();
                // Definitely shouldn't be inlining a proxy that takes
                // up more than 1 full treelet
                CHECK_EQ(readers.size(), 1);
                uint32_t numMeshes;
                readers[0].second->read(&numMeshes);
                numProxyMeshes += numMeshes;
            }
        }

        unsigned sTreeletID = _manager.getId(&treelet);
        auto writer = make_unique<LiteRecordWriter>(
            _manager.getFilePath(ObjectType::Treelet, sTreeletID));

        writer->write(static_cast<uint32_t>(0));  // numImgParts
        writer->write(static_cast<uint32_t>(0));  // numTexs
        writer->write(static_cast<uint32_t>(0));  // numStexs
        writer->write(static_cast<uint32_t>(0));  // numFtexs
        writer->write(static_cast<uint32_t>(0));  // numMats

        uint32_t numTriMeshes = 0;
        const auto numTriMeshesOffset = writer->offset();
        writer->write(numTriMeshes);

        unordered_map<TriangleMesh *,
                      unordered_map<size_t, pair<size_t, size_t>>>
            triNumRemap;  // mesh -> (triNum -> (newMesh, newTriNum))
        unordered_map<TriangleMesh *, uint32_t> triMeshIDs;

        // Write out rewritten meshes with only triangles in treelet
        for (auto &kv : trianglesInTreelet) {
            TriangleMesh *const mesh = kv.first;
            const vector<size_t> &triNums = kv.second;

            vector<shared_ptr<TriangleMesh>> meshesToWrite;

            shared_ptr<TriangleMesh> newMesh;
            const auto newMeshId = _manager.getNextId(ObjectType::TriangleMesh);

            if (!triNums.empty()) {
                newMesh = cutMesh(newMeshId, mesh, triNums, triNumRemap[mesh]);
            } else {
                throw runtime_error("we shouldn't get to this point");
            }

            const uint32_t mtlID = _manager.getMeshMaterialId(mesh);
            if (_manager.isCompoundMaterial(mtlID)) {
                throw runtime_error("not supported for proxydumpbvh");
            } else {
                triMeshIDs[newMesh.get()] = newMeshId;
                _manager.recordMeshMaterialId(newMesh.get(), mtlID);
                meshesToWrite.push_back(move(newMesh));
            }

            const uint32_t areaLightID = _manager.getMeshAreaLightId(mesh);

            LOG(INFO) << "Writing " << meshesToWrite.size()
                      << " triangle meshe(s).";

            for (auto &m : meshesToWrite) {
                numTriMeshes++;

                const auto sMeshID = triMeshIDs.at(m.get());
                const uint32_t mtlID = _manager.getMeshMaterialId(m.get());
                const auto mData = serdes::triangle_mesh::serialize(*m);

                MaterialKey mtlKey;
                mtlKey.treelet = _manager.getMaterialTreeletId(mtlID);
                mtlKey.id = mtlID;

                // writing the triangle mesh
                writer->write(static_cast<uint64_t>(sMeshID));
                writer->write(mtlKey);
                writer->write(areaLightID);
                writer->write(mData);
            }
        }

        // Write out the full triangle meshes for all the included
        // proxies referenced by this treelet
        unordered_map<const ProxyBVH *, unordered_map<uint32_t, uint32_t>>
            proxyMeshIndices;
        if (inlineProxies) {
            for (const ProxyBVH *proxy : treelet.proxies) {
                auto readers = proxy->GetReaders();
                CHECK_EQ(readers.size(), 1);

                auto &reader = readers[0].second;

                reader->skip(reader->read<uint32_t>());  // numImgParts
                reader->skip(reader->read<uint32_t>());  // numTexs
                reader->skip(reader->read<uint32_t>());  // numStexs
                reader->skip(reader->read<uint32_t>());  // numFtexs
                reader->skip(reader->read<uint32_t>());  // numMats
                const uint32_t numMeshes = reader->read<uint32_t>();

                for (int i = 0; i < numMeshes; i++) {
                    numTriMeshes++;

                    const auto oldId = reader->read<uint64_t>();
                    const auto mtlKey = reader->read<MaterialKey>();
                    const auto areaLightId = reader->read<uint32_t>();
                    const auto len = reader->next_record_size();

                    shared_ptr<char> storage{new char[len],
                                             default_delete<char[]>()};
                    reader->read(storage.get(), len);

                    auto m = make_shared<TriangleMesh>(storage, 0);
                    const uint32_t newId =
                        _manager.getNextId(ObjectType::TriangleMesh);

                    writer->write(static_cast<uint64_t>(newId));
                    writer->write(mtlKey);
                    writer->write(areaLightId);
                    writer->write(serdes::triangle_mesh::serialize(*m));

                    proxyMeshIndices[proxy].emplace(oldId, newId);
                }
            }
        }

        writer->write_at(numTriMeshesOffset, numTriMeshes);

        size_t current_primitive_offset = 0;
        vector<CloudBVH::TreeletNode> output_nodes;

        enum Child { LEFT = 0, RIGHT = 1 };
        stack<pair<uint32_t, Child>> q;

        // Write out nodes for treelet
        for (uint64_t nodeIdx : treelet.nodes) {
            const LinearBVHNode &node = nodes[nodeIdx];
            output_nodes.emplace_back(node.bounds, node.axis);

            auto &out_node = output_nodes.back();

            if (not q.empty()) {
                auto parent = q.top();
                q.pop();

                output_nodes[parent.first].child_treelet[parent.second] =
                    sTreeletID;
                output_nodes[parent.first].child_node[parent.second] =
                    output_nodes.size() - 1;
            }

            if (node.nPrimitives == 0) {  // it's not a leaf
                uint32_t r_tid =
                    treeletAllocations[treelet.dirIdx][node.secondChildOffset];
                if (r_tid != treeletID) {
                    out_node.child_treelet[RIGHT] =
                        _manager.getId(&allTreelets[r_tid]);
                    out_node.child_node[RIGHT] =
                        treeletNodeLocations[r_tid].at(node.secondChildOffset);
                } else {
                    q.emplace(output_nodes.size() - 1, RIGHT);
                }

                uint32_t l_tid =
                    treeletAllocations[treelet.dirIdx][nodeIdx + 1];
                if (l_tid != treeletID) {
                    out_node.child_treelet[LEFT] =
                        _manager.getId(&allTreelets[l_tid]);
                    out_node.child_node[LEFT] =
                        treeletNodeLocations[l_tid].at(nodeIdx + 1);
                } else {
                    q.emplace(output_nodes.size() - 1, LEFT);
                }
            } else {  // it is a leaf
                out_node.leaf_tag = ~0;
                out_node.primitive_offset = current_primitive_offset;
                out_node.primitive_count = node.nPrimitives;

                current_primitive_offset += node.nPrimitives;
            }
        }

        writer->write(static_cast<uint32_t>(output_nodes.size()));
        const auto primCountOffset = writer->offset();
        writer->write(static_cast<uint32_t>(0));

        writer->write(reinterpret_cast<const char *>(output_nodes.data()),
                      sizeof(CloudBVH::TreeletNode) * output_nodes.size());
        output_nodes.clear();

        uint32_t primitiveCount = 0;

        // Write out nodes for instances
        if (inlineProxies) {
            for (const ProxyBVH *proxy : treelet.proxies) {
                auto readers = proxy->GetReaders();
                auto &reader = readers[0].second;

                reader->skip(reader->read<uint32_t>());      // numImgParts
                reader->skip(reader->read<uint32_t>());      // numTexs
                reader->skip(reader->read<uint32_t>());      // numStexs
                reader->skip(reader->read<uint32_t>());      // numFtexs
                reader->skip(reader->read<uint32_t>());      // numMats
                reader->skip(4 * reader->read<uint32_t>());  // numMeshes

                const uint32_t proxy_node_count = reader->read<uint32_t>();
                const uint32_t proxy_primitive_count = reader->read<uint32_t>();
                vector<CloudBVH::TreeletNode> proxy_nodes;

                proxy_nodes.resize(proxy_node_count);
                reader->read(reinterpret_cast<char *>(&nodes[0]),
                             proxy_node_count * sizeof(CloudBVH::TreeletNode));

                for (auto &pnode : proxy_nodes) {
                    if (!pnode.is_leaf()) {
                        pnode.child_treelet[LEFT] = sTreeletID;
                        pnode.child_treelet[RIGHT] = sTreeletID;
                        pnode.child_node[LEFT] += output_nodes.size();
                        pnode.child_node[RIGHT] += output_nodes.size();
                    } else {
                        pnode.primitive_offset += current_primitive_offset;
                    }
                }

                output_nodes.insert(output_nodes.end(), proxy_nodes.begin(),
                                    proxy_nodes.end());
                current_primitive_offset += proxy_primitive_count;
            }
        }

        // Write out primitives for treelet
        serdes::cloudbvh::TransformedPrimitive primitive;
        serdes::cloudbvh::Triangle triangle;

        for (uint64_t nodeIdx : treelet.nodes) {
            const LinearBVHNode &node = nodes[nodeIdx];

            uint32_t transformed_primitive_count = 0;
            uint32_t triangle_count = 0;

            for (int i = 0; i < node.nPrimitives; i++) {
                if (primitives[node.primitivesOffset + i]->GetType() ==
                    PrimitiveType::Transformed) {
                    transformed_primitive_count++;
                } else {
                    triangle_count++;
                }
            }

            primitiveCount += transformed_primitive_count + triangle_count;
            writer->write(transformed_primitive_count);
            writer->write(triangle_count);

            // write all transformed primitives for the node
            for (int primIdx = 0; primIdx < node.nPrimitives; primIdx++) {
                auto &prim = primitives[node.primitivesOffset + primIdx];

                if (prim->GetType() == PrimitiveType::Transformed) {
                    shared_ptr<TransformedPrimitive> tp =
                        dynamic_pointer_cast<TransformedPrimitive>(prim);
                    shared_ptr<ProxyBVH> proxy =
                        dynamic_pointer_cast<ProxyBVH>(tp->GetPrimitive());

                    CHECK_NOTNULL(proxy.get());
                    uint64_t proxyRef;
                    if (inlineProxies) {
                        if (!largeProxies.count(proxy.get())) {
                            proxyRef = treeletID;
                            proxyRef <<= 32;
                            proxyRef |=
                                treeletProxyStarts[treeletID].at(proxy.get());
                        } else {
                            auto iter = nonCopyableProxyRoots.find(proxy.get());
                            CHECK_NE(iter == nonCopyableProxyRoots.end(), true);

                            proxyRef = iter->second[treelet.dirIdx];
                            proxyRef <<= 32;
                        }
                    } else {
                        proxyRef = proxyOrder.find(proxy.get())->second;
                        proxyRef <<= 32;
                    }

                    auto &t = tp->GetTransform();

                    primitive.root_ref = proxyRef;
                    primitive.start_transform = t.StartTransform()->GetMatrix();
                    primitive.end_transform = t.EndTransform()->GetMatrix();
                    primitive.start_time = t.StartTime();
                    primitive.end_time = t.EndTime();

                    writer->write(primitive);
                }
            }

            // write all triangles for the node
            for (int primIdx = 0; primIdx < node.nPrimitives; primIdx++) {
                auto &prim = primitives[node.primitivesOffset + primIdx];

                if (prim->GetType() != PrimitiveType::Transformed) {
                    shared_ptr<GeometricPrimitive> gp =
                        dynamic_pointer_cast<GeometricPrimitive>(prim);
                    const Shape *shape = gp->GetShape();
                    const Triangle *tri = dynamic_cast<const Triangle *>(shape);
                    CHECK_NOTNULL(tri);
                    TriangleMesh *mesh = tri->mesh.get();

                    int origTriNum = (tri->v - mesh->vertexIndices) / 3;
                    auto info = triNumRemap.at(mesh).at(origTriNum);

                    triangle.mesh_id = info.first;
                    triangle.tri_number = info.second;

                    writer->write(triangle);
                }
            }
        }

        // Write out primitives for instances
        if (inlineProxies) {
            for (const ProxyBVH *proxy : treelet.proxies) {
                auto readers = proxy->GetReaders();
                auto &reader = readers[0].second;

                reader->skip(reader->read<uint32_t>());      // numImgParts
                reader->skip(reader->read<uint32_t>());      // numTexs
                reader->skip(reader->read<uint32_t>());      // numStexs
                reader->skip(reader->read<uint32_t>());      // numFtexs
                reader->skip(reader->read<uint32_t>());      // numMats
                reader->skip(4 * reader->read<uint32_t>());  // numMeshes

                const uint32_t proxy_node_count = reader->read<uint32_t>();
                const uint32_t proxy_primitive_count = reader->read<uint32_t>();

                reader->skip(1);  // skip nodes
                for (size_t i = 0; i < proxy_node_count; i++) {
                    const uint32_t transformed_count = reader->read<uint32_t>();
                    const uint32_t triangle_count = reader->read<uint32_t>();

                    primitiveCount += transformed_count + triangle_count;

                    writer->write(transformed_count);
                    writer->write(triangle_count);

                    for (size_t i = 0; i < transformed_count; i++) {
                        reader->read(&primitive);
                        const uint32_t proxyIdx =
                            static_cast<uint32_t>(primitive.root_ref >> 32);
                        const ProxyBVH *dep = proxy->Dependencies()[proxyIdx];

                        uint64_t proxyRef = 0;
                        if (treeletProxyStarts[treeletID].count(dep)) {
                            proxyRef = treeletID;
                            proxyRef <<= 32;
                            proxyRef |= treeletProxyStarts[treeletID].at(dep);
                        } else {
                            CHECK_EQ(largeProxies.count(dep), 1);
                            proxyRef =
                                nonCopyableProxyRoots.at(dep)[treelet.dirIdx];
                            proxyRef <<= 32;
                        }

                        primitive.root_ref = proxyRef;
                        writer->write(primitive);
                    }

                    for (size_t i = 0; i < triangle_count; i++) {
                        reader->read(&triangle);
                        triangle.mesh_id =
                            proxyMeshIndices.at(proxy).at(triangle.mesh_id);
                        writer->write(triangle);
                    }
                }
            }
        }

        writer->write_at(primCountOffset, primitiveCount);
    }

    int numRoots = multiDir ? 8 : 1;

    vector<uint32_t> rootTreelets;
    for (int i = 0; i < numRoots; i++) {
        rootTreelets.push_back(_manager.getId(&allTreelets[i]));
    }

    return rootTreelets;
}

bool ProxyDumpBVH::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
    Error("Unimplemented");
    return false;
}

bool ProxyDumpBVH::IntersectP(const Ray &ray) const {
    Error("Unimplemented");
    return false;
}

}  // namespace pbrt
