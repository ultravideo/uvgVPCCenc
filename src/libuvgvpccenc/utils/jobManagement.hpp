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

#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>

#include "threadqueue.hpp"
#include "uvgvpcc/log.hpp"

#define JOBF(gofId, frameId, priority, func, ...) \
    uvgvpcc_enc::JobManager::make_job(gofId, frameId, priority, std::string(#func), func, ##__VA_ARGS__)

#define JOBG(gofId, priority, func, ...) uvgvpcc_enc::JobManager::make_job(gofId, priority, std::string(#func), func, ##__VA_ARGS__)

#define TO_STRING(x) #x

namespace uvgvpcc_enc {

class jobKey {
   private:
    size_t gofId_;
    std::optional<size_t> frameId_;
    std::string funcName_;

   public:
    jobKey(const size_t& gofId, const size_t& frameId, const std::string& funcName);
    jobKey(const size_t& gofId, const std::string& funcName);
    std::string toString() const;

    // Getters
    size_t getGofId() const;
    std::optional<size_t> getFrameId() const;
    std::string getFuncName() const;

    bool operator==(const jobKey& other) const;
};
}  // namespace uvgvpcc_enc

namespace std {
template <>
struct hash<uvgvpcc_enc::jobKey> {
    size_t operator()(const uvgvpcc_enc::jobKey& key) const;
};
}  // namespace std

namespace uvgvpcc_enc {

struct JobManager {
    static std::unique_ptr<ThreadQueue> threadQueue;
    static std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> previousGOFJobMap;
    static std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> previousFrameJobMap;
    static std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> currentGOFJobMap;
    static std::unique_ptr<std::unordered_map<jobKey, std::shared_ptr<Job>>> currentFrameJobMap;

    template <typename Func, typename... Args>
    static std::shared_ptr<Job> make_job(const size_t& gofId, const size_t& frameId, std::size_t priority, std::string funcName, Func&& func,
                                         Args&&... args);

    template <typename Func, typename... Args>
    static std::shared_ptr<Job> make_job(const size_t& gofId, std::size_t priority, std::string funcName, Func&& func, Args&&... args);

    static std::shared_ptr<Job> getJob(size_t gofId, size_t frameId, const std::string& funcName);
    static std::shared_ptr<Job> getJob(size_t gofId, const std::string& funcName);

    static void initThreadQueue(uint16_t numThreads);

    static void submitCurrentFrameJobs();

    static void submitCurrentGOFJobs();
};
}  // namespace uvgvpcc_enc

// Include template implementations
#include "jobManagement.tpp"
