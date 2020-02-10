#include "cloud/lambda-worker.h"
#include "messages/utils.h"

using namespace std;
using namespace chrono;
using namespace pbrt;
using namespace meow;
using namespace PollerShortNames;

using OpCode = Message::OpCode;

ResultType LambdaWorker::handleOutQueue() {
    bernoulli_distribution bd{config.bagLogRate};

    auto createNewBag = [&](const TreeletId treeletId) -> RayBag {
        RayBag bag{*workerId, treeletId, currentBagId[treeletId]++, false,
                   MAX_BAG_SIZE};

        bag.info.tracked = bd(randEngine);
        logBag(BagAction::Created, bag.info);

        if (!sealBagsTimer.armed()) {
            sealBagsTimer.set(0s, SEAL_BAGS_INTERVAL);
        }

        return bag;
    };

    for (auto it = outQueue.begin(); it != outQueue.end();
         it = outQueue.erase(it)) {
        const TreeletId treeletId = it->first;
        auto& rayList = it->second;

        auto bagIt = openBags.find(treeletId);

        if (bagIt == openBags.end()) {
            auto result = openBags.emplace(treeletId, createNewBag(treeletId));
            bagIt = result.first;
        }

        while (!rayList.empty()) {
            auto& ray = rayList.front();
            auto& bag = bagIt->second;

            if (bag.info.bagSize + ray->MaxCompressedSize() > MAX_BAG_SIZE) {
                logBag(BagAction::Sealed, bag.info);
                sealedBags.push(move(bag));

                /* let's create an empty bag */
                bag = createNewBag(treeletId);
            }

            const auto len = ray->Serialize(&bag.data[0] + bag.info.bagSize);
            bag.info.rayCount++;
            bag.info.bagSize += len;

            logRay(RayAction::Bagged, *ray, bag.info);

            rayList.pop();
            outQueueSize--;
        }
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleOpenBags() {
    sealBagsTimer.read_event();

    nanoseconds nextExpiry = nanoseconds::max();
    const auto now = steady_clock::now();

    for (auto it = openBags.begin(); it != openBags.end();) {
        const auto timeSinceCreation = now - it->second.createdAt;

        if (timeSinceCreation < SEAL_BAGS_INTERVAL) {
            it++;
            nextExpiry = min(nextExpiry,
                             1ns + duration_cast<nanoseconds>(
                                       SEAL_BAGS_INTERVAL - timeSinceCreation));

            continue;
        }

        logBag(BagAction::Sealed, it->second.info);
        sealedBags.push(move(it->second));
        it = openBags.erase(it);
    }

    if (!openBags.empty()) {
        sealBagsTimer.set(0s, nextExpiry);
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleSamples() {
    auto& out = sampleBags;

    if (out.empty()) {
        out.emplace(*workerId, 0, currentSampleBagId++, true, MAX_BAG_SIZE);
    }

    while (!samples.empty()) {
        auto& sample = samples.front();

        if (out.back().info.bagSize + sample.MaxCompressedSize() >
            MAX_BAG_SIZE) {
            out.emplace(*workerId, 0, currentSampleBagId++, true, MAX_BAG_SIZE);
        }

        auto& bag = out.back();
        const auto len = sample.Serialize(&bag.data[0] + bag.info.bagSize);
        bag.info.rayCount++;
        bag.info.bagSize += len;

        samples.pop();
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleSealedBags() {
    while (!sealedBags.empty()) {
        auto& bag = sealedBags.front();

        bag.data.erase(bag.info.bagSize);
        bag.data.shrink_to_fit();
        logBag(BagAction::Submitted, bag.info);

        const auto id = transferAgent->requestUpload(
            bag.info.str(rayBagsKeyPrefix), move(bag.data));

        pendingRayBags[id] = make_pair(Task::Upload, bag.info);
        sealedBags.pop();
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleSampleBags() {
    sampleBagsTimer.read_event();

    while (!sampleBags.empty()) {
        RayBag& bag = sampleBags.front();
        bag.data.erase(bag.info.bagSize);

        const auto id = samplesTransferAgent->requestUpload(
            bag.info.str(rayBagsKeyPrefix), move(bag.data));

        pendingRayBags[id] = make_pair(Task::Upload, bag.info);
        sampleBags.pop();
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleReceiveQueue() {
    while (!receiveQueue.empty()) {
        RayBag bag = move(receiveQueue.front());
        receiveQueue.pop();

        /* (1) XXX do we have this treelet? */

        /* (2) let's unpack this treelet and add the rays to the trace queue */
        const char* data = bag.data.data();
        auto& rays = traceQueue[bag.info.treeletId];

        for (size_t offset = 0; offset < bag.data.size();) {
            const auto len = *reinterpret_cast<const uint32_t*>(data + offset);
            offset += 4;

            RayStatePtr ray = RayState::Create();
            ray->Deserialize(data + offset, len);
            ray->hop++;
            ray->pathHop++;
            offset += len;

            logRay(RayAction::Unbagged, *ray, bag.info);

            rays.push(move(ray));
        }

        logBag(BagAction::Opened, bag.info);
    }

    return ResultType::Continue;
}

ResultType LambdaWorker::handleTransferResults(const bool sampleBags) {
    protobuf::RayBags enqueuedProto;
    protobuf::RayBags dequeuedProto;

    auto& agent = sampleBags ? samplesTransferAgent : transferAgent;

    if (!agent->eventfd().read_event()) {
        return ResultType::Continue;
    }

    vector<pair<uint64_t, string>> actions;
    agent->tryPopBulk(back_inserter(actions));

    for (auto& action : actions) {
        auto infoIt = pendingRayBags.find(action.first);

        if (infoIt != pendingRayBags.end()) {
            const auto& info = infoIt->second.second;

            switch (infoIt->second.first) {
            case Task::Upload: {
                /* we have to tell the master that we uploaded this */
                *enqueuedProto.add_items() = to_protobuf(info);
                activeRays -= info.rayCount;

                logBag(BagAction::Enqueued, info);
                break;
            }

            case Task::Download:
                /* we have to put the received bag on the receive queue,
                   and tell the master */
                receiveQueue.emplace(info, move(action.second));
                *dequeuedProto.add_items() = to_protobuf(info);
                activeRays += info.rayCount;

                logBag(BagAction::Dequeued, info);
                break;
            }

            pendingRayBags.erase(infoIt);
        }
    }

    enqueuedProto.set_active_rays(activeRays);
    dequeuedProto.set_active_rays(activeRays);

    if (enqueuedProto.items_size() > 0) {
        coordinatorConnection->enqueue_write(
            Message::str(*workerId, OpCode::RayBagEnqueued,
                         protoutil::to_string(enqueuedProto)));
    }

    if (dequeuedProto.items_size() > 0) {
        coordinatorConnection->enqueue_write(
            Message::str(*workerId, OpCode::RayBagDequeued,
                         protoutil::to_string(dequeuedProto)));
    }

    return ResultType::Continue;
}
