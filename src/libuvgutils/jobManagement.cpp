/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024-present, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

#include "uvgutils/jobManagement.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "uvgutils/log.hpp"
#include "uvgutils/threadqueue.hpp"

namespace {
std::shared_ptr<uvgutils::Job> getJob_impl(const uvgutils::jobKey& key) {
    std::shared_ptr<uvgutils::Job> job = nullptr;
    // Check if it's a frame job or GOF job
    if (key.getFrameId().has_value()) {
        // Check current frame jobs first
        if (uvgutils::JobManager::currentFrameJobMap) {
            auto it = uvgutils::JobManager::currentFrameJobMap->find(key);
            if (it != uvgutils::JobManager::currentFrameJobMap->end()) {
                job = it->second;
            }
        }
        // Check previous frame jobs if not found
        if (!job && uvgutils::JobManager::previousFrameJobMap) {
            auto it = uvgutils::JobManager::previousFrameJobMap->find(key);
            if (it != uvgutils::JobManager::previousFrameJobMap->end()) {
                job = it->second;
            }
        }
    } else {
        // Check current GOF jobs first
        if (uvgutils::JobManager::currentGOFJobMap) {
            auto it = uvgutils::JobManager::currentGOFJobMap->find(key);
            if (it != uvgutils::JobManager::currentGOFJobMap->end()) {
                job = it->second;
            }
        }
        // Check previous GOF jobs if not found
        if (!job && uvgutils::JobManager::previousGOFJobMap) {
            auto it = uvgutils::JobManager::previousGOFJobMap->find(key);
            if (it != uvgutils::JobManager::previousGOFJobMap->end()) {
                job = it->second;
            }
        }
    }

    if (job) {
        uvgutils::Logger::log<uvgutils::LogLevel::DEBUG>("JOB FACTORY", key.toString() + " Job found\n");
        return job;
    }
    uvgutils::Logger::log<uvgutils::LogLevel::WARNING>("JOB FACTORY", key.toString() + " Job not found\n");
    return nullptr;
}
}  // namespace

namespace uvgutils {

// Static member definitions
std::unique_ptr<ThreadQueue> JobManager::threadQueue = nullptr;
std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> JobManager::previousGOFJobMap = nullptr;
std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> JobManager::previousFrameJobMap = nullptr;
std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> JobManager::currentGOFJobMap = nullptr;
std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> JobManager::currentFrameJobMap = nullptr;

// jobKey constructors
jobKey::jobKey(const size_t& gofId, const size_t& frameId, const std::string& funcName)
    : gofId_(gofId), frameId_(frameId), funcName_(funcName) {}

jobKey::jobKey(const size_t& gofId, const std::string& funcName) : gofId_(gofId), frameId_(std::nullopt), funcName_(funcName) {}

// jobKey methods
std::string jobKey::toString() const {
    if (frameId_.has_value()) {
        return "[GOF " + std::to_string(gofId_) + "][Frame " + std::to_string(frameId_.value()) + "][" + funcName_ + "]";
    }
    return "[GOF " + std::to_string(gofId_) + "][" + funcName_ + "]";
}

size_t jobKey::getGofId() const { return gofId_; }

std::optional<size_t> jobKey::getFrameId() const { return frameId_; }

std::string jobKey::getFuncName() const { return funcName_; }

bool jobKey::operator==(const jobKey& other) const {
    return gofId_ == other.gofId_ && frameId_ == other.frameId_ && funcName_ == other.funcName_;
}

// JobManager methods
std::shared_ptr<Job> JobManager::getJob(size_t gofId, size_t frameId, const std::string& funcName) {
    const jobKey key(gofId, frameId, funcName);
    return getJob_impl(key);
}

std::shared_ptr<Job> JobManager::getJob(size_t gofId, const std::string& funcName) {
    const jobKey key(gofId, funcName);
    return getJob_impl(key);
}

// JobManager method implementations
void JobManager::initThreadQueue(uint16_t numThreads) {
    threadQueue = std::make_unique<ThreadQueue>();
    threadQueue->initThreadQueue(numThreads);
    currentGOFJobMap = std::make_unique<std::unordered_map<jobKey, std::shared_ptr<Job>>>();
    currentFrameJobMap = std::make_unique<std::unordered_map<jobKey, std::shared_ptr<Job>>>();
}

void JobManager::submitCurrentFrameJobs() {
    if (currentFrameJobMap) {
        for (const auto& jobPair : *currentFrameJobMap) {
            threadQueue->submitJob(jobPair.second);
        }
    }
    previousFrameJobMap = std::move(currentFrameJobMap);
    currentFrameJobMap = std::make_unique<std::unordered_map<jobKey, std::shared_ptr<Job>>>();
}

void JobManager::submitCurrentGOFJobs() {
    if (currentGOFJobMap) {
        for (const auto& jobPair : *currentGOFJobMap) {
            threadQueue->submitJob(jobPair.second);
        }
    }
    previousGOFJobMap = std::move(currentGOFJobMap);
    currentGOFJobMap = std::make_unique<std::unordered_map<jobKey, std::shared_ptr<Job>>>();
}

}  // namespace uvgutils

// Hash function implementation
namespace std {
size_t hash<uvgutils::jobKey>::operator()(const uvgutils::jobKey& key) const {
    const size_t h1 = std::hash<size_t>{}(key.getGofId());
    const size_t h2 = key.getFrameId().has_value() ? std::hash<size_t>{}(key.getFrameId().value()) : 0;
    const size_t h3 = std::hash<std::string>{}(key.getFuncName());
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}
}  // namespace std