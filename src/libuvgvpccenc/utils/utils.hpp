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

/// \file Common tools used by the uvgVPCCenc library.


#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <sstream>
#include <iomanip>

namespace uvgvpcc_enc {

using typeGeometryInput = uint16_t;

const typeGeometryInput g_infiniteDepth = (std::numeric_limits<typeGeometryInput>::max)();  // TODO(lf)be sure it is well sync with type geo
const size_t g_infinitenumber = (std::numeric_limits<size_t>::max)();
const size_t g_valueNotSet = (std::numeric_limits<size_t>::max)();

constexpr size_t INVALID_PATCH_INDEX = std::numeric_limits<size_t>::max();
constexpr size_t PPI_NON_ASSIGNED = std::numeric_limits<size_t>::max();
constexpr size_t UNDEFINED_PARENT_PPI = std::numeric_limits<size_t>::max() - 1; //TODO(lf) temp

// Projection Plan Index, 0-5 -> one of the six bounding box plan. 6+ -> used for slicing ppi attribution
enum class PPI : uint8_t {ppi0,ppi1,ppi2,ppi3,ppi4,ppi5,ppiBlank,notAssigned};

template <typename T>
class Vector3 : public std::array<T, 3> {
   public:
    Vector3() : std::array<T, 3>() {}
    Vector3(T x, T y, T z) : std::array<T, 3>({x, y, z}) {}
    Vector3(std::array<T, 3>& arr) : std::array<T, 3>(arr) {}
    Vector3(std::array<T, 3>&& arr) : std::array<T, 3>(std::move(arr)) {} 
    Vector3(const std::array<T, 3>& arr) {
        std::copy(arr.begin(), arr.end(), this->begin());
    }   
    
    template <typename U>
    Vector3<T> operator+(const Vector3<U>& other) const {
        return {(*this)[0] + other[0], (*this)[1] + other[1], (*this)[2] + other[2]};
    }

    template <typename U>
    Vector3<T> operator-(const Vector3<U>& other) const {
        return {(*this)[0] - other[0], (*this)[1] - other[1], (*this)[2] - other[2]};
    }

    Vector3<double> operator-(const Vector3<double>& other) const {
        return {(*this)[0] - other[0], (*this)[1] - other[1], (*this)[2] - other[2]};
    }

    Vector3<T> operator-() const { return {-(*this)[0], -(*this)[1], -(*this)[2]}; }

    template <typename U>
    Vector3<T>& operator+=(const Vector3<U>& other) {
        (*this)[0] += other[0];
        (*this)[1] += other[1];
        (*this)[2] += other[2];
        return *this;
    }

    template <typename U>
    Vector3<T>& operator/=(const U& val) {
        (*this)[0] /= val;
        (*this)[1] /= val;
        (*this)[2] /= val;
        return *this;
    }
};

inline std::string zeroPad(size_t value, size_t width) {
    std::ostringstream oss;
    oss << std::setw(width) << std::setfill('0') << value;
    return oss.str();
};

// Round the given number to the nearest bigger multiple.
// equivalent to : return = ceil(numberF/multipleF) * multiple;
// Examples : roundUp(7,8) = 8;  roundUp(17,8) = 24;
inline size_t roundUp(const size_t& number, const size_t& multiple) { return (number + multiple - 1) & -multiple;}

} // uvgvpcc_enc namespace