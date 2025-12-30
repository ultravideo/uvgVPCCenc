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

/// \file Custom logging class that enables fine-tuning of the log level and retrieval of Kvazaar's own logs.

#include <chrono>
#include <cstdarg>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

#ifndef UVG_LOG_LEVEL
#define UVG_LOG_LEVEL LogLevel::DEBUG
#endif

namespace uvgutils {

enum class LogLevel { FATAL, ERROR, WARNING, INFO, PROFILING, TRACE, DEBUG };
constexpr LogLevel COMPILETIME_LOG_LEVEL = UVG_LOG_LEVEL;

// stringify table for LogLevel
static const std::string LogLevelStr[] = {"FATAL", "ERROR", "WARNING", "INFO", "PROFILING", "TRACE", "DEBUG"};

constexpr bool errorsAreFatalDefaultValue = true;
constexpr LogLevel logLevelDefaultValue = LogLevel::INFO;
constexpr std::ostream* outputDefaultValue = &std::cerr;

class Logger {
   public:
    static void setLogLevel(const LogLevel& level);
    static void setErrorsAreFatal(const bool& isFatal);
    static void setOutputStream(std::ostream& out);
    static LogLevel getLogLevel();
    static void printLogMessage(const std::string& context, LogLevel level, const std::string& message);

    template <LogLevel LEVEL>
    static void log(const std::string& context, const std::string& message) {
        static std::mutex logMutex;
        const std::lock_guard<std::mutex> lock(logMutex);
        if constexpr (LEVEL <= COMPILETIME_LOG_LEVEL) {
            if (LEVEL <= Logger::getLogLevel()) {
                printLogMessage(context, LEVEL, message);
            }
        }
    }
    static std::string printfStrToStdStr(const char* fmt, ...);
    static std::string vprintfStrToStdStr(const char* fmt, va_list args);

   private:
    static LogLevel logLevel;
    static bool errorsAreFatal_;
    static std::ostream* outputStream_;
};

template <LogLevel LEVEL>
inline void log(const std::string& context, const std::string& message) {
    Logger::log<LEVEL>(context, message);
}

class Timer {
   public:
    Timer() : start(std::chrono::steady_clock::now()) {}

    // Function to get elapsed time since the start of the program
    double elapsed() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count() / 1000000.0;
    }
    std::string elapsed_str() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << elapsed();
        return oss.str();
    }

   private:
    std::chrono::time_point<std::chrono::steady_clock> start;
};

static Timer global_timer;

}  // namespace uvgutils
