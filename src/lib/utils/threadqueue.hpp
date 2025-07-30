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

/// \file Custom thread pool implementation based on the Kvazaar own implementation.

#pragma once

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace uvgvpcc_enc {

enum threadqueue_job_state {
    /**
     * \brief Job has been submitted, but is not allowed to run yet.
     */
    THREADQUEUE_JOB_STATE_PAUSED,

    /**
     * \brief Job is waiting for dependencies.
     */
    THREADQUEUE_JOB_STATE_WAITING,

    /**
     * \brief Job is ready to run.
     */
    THREADQUEUE_JOB_STATE_READY,

    /**
     * \brief Job is running.
     */
    THREADQUEUE_JOB_STATE_RUNNING,

    /**
     * \brief Job is completed.
     */
    THREADQUEUE_JOB_STATE_DONE,
};

class ThreadQueue;

class Job : public std::enable_shared_from_this<Job> {
   public:
    using JobFunction = std::function<void()>;
    Job(std::string name, std::size_t priority, JobFunction func)
        : name_(name),
          func_(func),
          state_(threadqueue_job_state::THREADQUEUE_JOB_STATE_PAUSED),
          dependencies_(0),
          priority(priority),
          completed_(false) {}
    // Variadic template constructor
    template <typename Func, typename... Args>
    Job(std::string name, std::size_t priority, Func&& func, Args&&... args)
        : name_(name),
          func_(std::bind(std::forward<Func>(func), std::forward<Args>(args)...)),
          state_(threadqueue_job_state::THREADQUEUE_JOB_STATE_PAUSED),
          dependencies_(0),
          priority(priority),
          completed_(false) {}
    void execute() const;
    void addDependency(const std::shared_ptr<Job>& dependency);
    bool isReady() const;
    void wait();
    void complete();
    std::string getName() const { return name_; }
    threadqueue_job_state getState() const { return state_; }
    void setState(threadqueue_job_state state) { state_ = state; }

    mutable std::mutex mtx_;
    std::vector<std::shared_ptr<Job>> reverseDependencies_;
    std::string name_;
    JobFunction func_;
    threadqueue_job_state state_;
    std::atomic<int> dependencies_;
    std::atomic<std::size_t> priority;

   private:
    std::condition_variable cv_;
    std::atomic<bool> completed_;
};

class ThreadQueue {
   public:
    ThreadQueue() : stop_(false) {};
    void initThreadQueue(int numThreads);
    ~ThreadQueue();

    void submitJob(const std::shared_ptr<Job>& job);
    void pushJob(const std::shared_ptr<Job>& job);
    void stop();
    static void waitForJob(const std::shared_ptr<Job>& job);

   private:
    void workerThread();
    std::mutex mtx_;
    std::condition_variable jobAvailable_;
    std::condition_variable jobDone_;
    std::vector<std::thread> threads_;
    std::array<std::deque<std::shared_ptr<Job>>, 6> jobs_;
    std::atomic<bool> stop_;
};

std::string jobStateToStr(threadqueue_job_state s);

}  // namespace uvgvpcc_enc