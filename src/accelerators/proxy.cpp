#include "accelerators/proxy.h"

#include "cloud/manager.h"

using namespace std;

namespace pbrt {

vector<pair<uint32_t, std::unique_ptr<FileRecordReader>>> ProxyBVH::GetReaders()
    const {
    vector<pair<uint32_t, std::unique_ptr<FileRecordReader>>> readers;
    vector<pair<uint32_t, std::unique_ptr<FileRecordReader>>> materialReaders;

    SceneManager mgr;
    mgr.init(PbrtOptions.proxyDir + "/" + name_);

    const uint32_t numTreelets = mgr.treeletCount();

    for (uint32_t i = 0; i < numTreelets; i++) {
        auto treeletPath = mgr.getFilePath(ObjectType::Treelet, i);
        FileRecordReader reader{treeletPath};

        /* is this a material treelet? */
        const auto numImgParts = reader.read<uint32_t>();  // numImgParts
        reader.skip(numImgParts);
        const auto numTexs = reader.read<uint32_t>();  // numTexs
        reader.skip(numTexs);
        const auto numStexs = reader.read<uint32_t>();  // numStexs
        reader.skip(numStexs);
        const auto numFtexs = reader.read<uint32_t>();  // numFtexs
        reader.skip(numFtexs);
        const auto numMats = reader.read<uint32_t>();  // numMats
        reader.skip(numMats);

        if (numImgParts || numTexs || numFtexs || numStexs || numMats) {
            materialReaders.emplace_back(
                i, make_unique<FileRecordReader>(treeletPath));
        } else {
            readers.emplace_back(i, make_unique<FileRecordReader>(treeletPath));
        }
    }

    readers.insert(readers.end(), make_move_iterator(materialReaders.begin()),
                   make_move_iterator(materialReaders.end()));

    return readers;
}

}  // namespace pbrt
