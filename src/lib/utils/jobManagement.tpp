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

// Template implementations for JobManager

namespace {
template <typename Func, typename... Args>
std::shared_ptr<uvgvpcc_enc::Job> make_job_impl(const uvgvpcc_enc::jobKey key, std::size_t priority, Func&& func, Args&&... args) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::DEBUG>("JOB FACTORY",
                                                           key.toString() + " Creating job with priority " + std::to_string(priority) + "\n");
    auto job = std::make_shared<uvgvpcc_enc::Job>(key.toString(), priority, std::bind(std::forward<Func>(func), std::forward<Args>(args)...));

    // Add job to appropriate map based on whether it has frameId
    if (key.getFrameId().has_value()) {
        if (uvgvpcc_enc::JobManager::currentFrameJobMap) {
            uvgvpcc_enc::JobManager::currentFrameJobMap->emplace(key, job);
        }
    } else {
        if (uvgvpcc_enc::JobManager::currentGOFJobMap) {
            uvgvpcc_enc::JobManager::currentGOFJobMap->emplace(key, job);
        }
    }

    return job;
}
}  // namespace

namespace uvgvpcc_enc {

template <typename Func, typename... Args>
std::shared_ptr<Job> JobManager::make_job(const size_t& gofId, const size_t& frameId, std::size_t priority, std::string funcName, Func&& func,
                                          Args&&... args) {
    return make_job_impl(jobKey(gofId, frameId, std::move(funcName)), priority, std::forward<Func>(func), std::forward<Args>(args)...);
}

template <typename Func, typename... Args>
std::shared_ptr<Job> JobManager::make_job(const size_t& gofId, std::size_t priority, std::string funcName, Func&& func, Args&&... args) {
    return make_job_impl(jobKey(gofId, std::move(funcName)), priority, std::forward<Func>(func), std::forward<Args>(args)...);
}

}  // namespace uvgvpcc_enc
