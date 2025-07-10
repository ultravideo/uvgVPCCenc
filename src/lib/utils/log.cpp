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

/// \file Custom logging class that enables fine-tuning of the log level and retrieval of Kvazaar's own logs.


#include "uvgvpcc/log.hpp"

// NOLINTNEXTLINE(hicpp-deprecated-headers)
#include <stdio.h>  // Needed for vasprintf

#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <regex>
#include <sstream>
#include <string>

namespace uvgvpcc_enc {

// Define color codes using const char*
constexpr const char* RED = "\x1B[31m";
constexpr const char* YEL = "\x1B[33m";
constexpr const char* BLU = "\x1B[34m";
constexpr const char* CYN = "\x1B[36m";
constexpr const char* GRN = "\x1B[32m";
constexpr const char* MAG = "\x1B[35m";
constexpr const char* BLD = "\x1B[1m";
constexpr const char* RST = "\x1B[0m";
constexpr const char* REDANDBLD = "\x1B[31m\x1B[1m";

// Define an array of concatenated literal strings
constexpr std::array<const char*, 7> log_color = {REDANDBLD, RED, YEL, BLU, CYN, GRN, MAG};

LogLevel Logger::logLevel = LogLevel::INFO;
bool Logger::errorsAreFatal_ = true;

namespace {

inline std::string getLogPrefix(const std::string& context, LogLevel level) {
    const std::string elapsedStr = global_timer.elapsed_str();
    return "[" + elapsedStr + "][" + LogLevelStr[static_cast<int>(level)] + "] [" + context + "] ";
}
}  // anonymous namespace

void Logger::setLogLevel(const LogLevel& level) { logLevel = level; }
void Logger::setErrorsAreFatal(const bool& isFatal) { errorsAreFatal_ = isFatal; }
LogLevel Logger::getLogLevel() { return logLevel; }

void Logger::log(LogLevel level, const std::string& context, const std::string& message) {
    static std::mutex logMutex;
    const std::lock_guard<std::mutex> lock(logMutex);

    if (level <= Logger::getLogLevel()) {
        static bool is_newline = true;
        std::ostringstream oss;
        if (is_newline) {
            oss << getLogPrefix(context, level);
            is_newline = false;
        }
        const bool last_is_newline = !message.empty() ? message.back() == '\n' : false;
        if (message.find('\n') != std::string::npos && !last_is_newline) {
            oss << std::regex_replace(message, std::regex(R"(\n(?!$))"), "\n" + getLogPrefix(context, level));
        } else {
            oss << message;
        }

        if (last_is_newline) {
            is_newline = true;
        }

        std::cerr << log_color[static_cast<int>(level)] << oss.str() << RST;

        // TODO(lf) : 
        // if (level == FATAL) : throw std::runtime_exception("");
        // if (level == ERROR && errorAreFatal): throw std::runtime_exception("");
        // display message, if you want to not stop the program ... set errorsAreFatal=false
    }
}

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cert-dcl50-cpp,cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay,cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)
std::string Logger::printfStrToStdStr(const char* fmt, ...) {
    char* str = nullptr;
    va_list args;
    va_start(args, fmt);
    if (vasprintf(&str, fmt, args) == -1) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::ERROR, "LOGGER", "vasprintf error in printfStrToStdStr function.\n");
        if(errorsAreFatal_) {throw std::runtime_error("");}

    }
    va_end(args);
    std::string result(str);
    free(str);  // Free the allocated memory
    return result;
}

std::string Logger::vprintfStrToStdStr(const char* fmt, va_list args) {
    char* str = nullptr;
    if (vasprintf(&str, fmt, args) == -1) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::ERROR, "LOGGER", "vasprintf error in vprintfStrToStdStr function.\n");
        if(errorsAreFatal_) {throw std::runtime_error("");}
    }
    std::string result(str);
    free(str);  // Free the allocated memory
    return result;
}
// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cert-dcl50-cpp,cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay,cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)

}  // namespace uvgvpcc_enc