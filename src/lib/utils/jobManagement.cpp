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

#include "jobManagement.hpp"

#include "threadqueue.hpp"
#include "uvgvpcc/log.hpp"

namespace {
std::shared_ptr<uvgvpcc_enc::Job> getJob_impl(const uvgvpcc_enc::jobKey key) {
    std::shared_ptr<uvgvpcc_enc::Job> job = nullptr;

    // Check if it's a frame job or GOF job
    if (key.getFrameId().has_value()) {
        // Check current frame jobs first
        if (uvgvpcc_enc::jobManager::currentFrameJobMap) {
            auto it = uvgvpcc_enc::jobManager::currentFrameJobMap->find(key);
            if (it != uvgvpcc_enc::jobManager::currentFrameJobMap->end()) {
                job = it->second;
            }
        }
        // Check previous frame jobs if not found
        if (!job && uvgvpcc_enc::jobManager::previousFrameJobMap) {
            auto it = uvgvpcc_enc::jobManager::previousFrameJobMap->find(key);
            if (it != uvgvpcc_enc::jobManager::previousFrameJobMap->end()) {
                job = it->second;
            }
        }
    } else {
        // Check current GOF jobs first
        if (uvgvpcc_enc::jobManager::currentGOFJobMap) {
            auto it = uvgvpcc_enc::jobManager::currentGOFJobMap->find(key);
            if (it != uvgvpcc_enc::jobManager::currentGOFJobMap->end()) {
                job = it->second;
            }
        }
        // Check previous GOF jobs if not found
        if (!job && uvgvpcc_enc::jobManager::previousGOFJobMap) {
            auto it = uvgvpcc_enc::jobManager::previousGOFJobMap->find(key);
            if (it != uvgvpcc_enc::jobManager::previousGOFJobMap->end()) {
                job = it->second;
            }
        }
    }

    if (job) {
        uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("JOB FACTORY", key.toString() + " Job found\n");
        return job;
    }
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::WARNING>("JOB FACTORY", key.toString() + " Job not found\n");
    return nullptr;
}
}  // namespace

namespace uvgvpcc_enc {

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

// jobManager methods
std::shared_ptr<Job> jobManager::getJob(size_t gofId, size_t frameId, const std::string& funcName) {
    jobKey key(gofId, frameId, funcName);
    return getJob_impl(key);
}

std::shared_ptr<Job> jobManager::getJob(size_t gofId, const std::string& funcName) {
    jobKey key(gofId, funcName);
    return getJob_impl(key);
}

// Static member definitions
ThreadQueue jobManager::threadQueue;
std::unordered_map<jobKey, std::shared_ptr<Job>>* jobManager::previousGOFJobMap = nullptr;
std::unordered_map<jobKey, std::shared_ptr<Job>>* jobManager::previousFrameJobMap = nullptr;
std::unordered_map<jobKey, std::shared_ptr<Job>>* jobManager::currentGOFJobMap = nullptr;
std::unordered_map<jobKey, std::shared_ptr<Job>>* jobManager::currentFrameJobMap = nullptr;

// jobManager method implementations
void jobManager::initThreadQueue(uint16_t numThreads) {
    threadQueue.initThreadQueue(numThreads);
    currentGOFJobMap = new std::unordered_map<jobKey, std::shared_ptr<Job>>();
    currentFrameJobMap = new std::unordered_map<jobKey, std::shared_ptr<Job>>();
}

void jobManager::submitCurrentFrameJobs() {
    if (currentFrameJobMap) {
        for (const auto& jobPair : *currentFrameJobMap) {
            threadQueue.submitJob(jobPair.second);
        }
    }
    if (previousFrameJobMap) {
        delete previousFrameJobMap;
    }
    previousFrameJobMap = currentFrameJobMap;
    currentFrameJobMap = new std::unordered_map<jobKey, std::shared_ptr<Job>>();
}

void jobManager::submitCurrentGOFJobs() {
    if (currentGOFJobMap) {
        for (const auto& jobPair : *currentGOFJobMap) {
            threadQueue.submitJob(jobPair.second);
        }
    }
    if (previousGOFJobMap) {
        delete previousGOFJobMap;
    }
    previousGOFJobMap = currentGOFJobMap;
    currentGOFJobMap = new std::unordered_map<jobKey, std::shared_ptr<Job>>();
}

}  // namespace uvgvpcc_enc

// Hash function implementation
namespace std {
size_t hash<uvgvpcc_enc::jobKey>::operator()(const uvgvpcc_enc::jobKey& key) const {
    size_t h1 = std::hash<size_t>{}(key.getGofId());
    size_t h2 = key.getFrameId().has_value() ? std::hash<size_t>{}(key.getFrameId().value()) : 0;
    size_t h3 = std::hash<std::string>{}(key.getFuncName());
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}
}  // namespace std