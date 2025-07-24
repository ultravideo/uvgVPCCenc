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

/// \file Entry point for computing the normals of a point cloud frame.

#include "normalComputation.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"
#include "utils/fileExport.hpp"

using namespace uvgvpcc_enc;


namespace {

void computeCovMat(std::array<uvgvpcc_enc::Vector3<double>, 3>& covMat, const uvgvpcc_enc::Vector3<double>& bary, const size_t nnCount,
                   const std::vector<size_t>& nnIndices, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry) {
    
    uvgvpcc_enc::Vector3<double> pt;
    for (size_t i = 0; i < nnCount; ++i) {
        pt = pointsGeometry[nnIndices[i]] - bary;

        covMat[0][0] += pt[0] * pt[0];
        covMat[1][1] += pt[1] * pt[1];
        covMat[2][2] += pt[2] * pt[2];
        covMat[0][1] += pt[0] * pt[1];
        covMat[0][2] += pt[0] * pt[2];
        covMat[1][2] += pt[1] * pt[2];
    }

    const double nnCountd = static_cast<double>(nnCount);
    covMat[0][0] /= (nnCountd - 1.0);
    covMat[0][1] /= (nnCountd - 1.0);
    covMat[0][2] /= (nnCountd - 1.0);
    covMat[1][1] /= (nnCountd - 1.0);
    covMat[1][2] /= (nnCountd - 1.0);
    covMat[2][2] /= (nnCountd - 1.0);

    covMat[1][0] = covMat[0][1];
    covMat[2][0] = covMat[0][2];
    covMat[2][1] = covMat[1][2];
}


// TMC2 implementation and comments //
// Slightly modified version of http://www.melax.com/diag.html?attredirects=0
// A must be a symmetric matrix.
// returns Q and D such that
// Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
// NOLINTBEGIN(cppcoreguidelines-init-variables)
void diagonalize(const std::array<uvgvpcc_enc::Vector3<double>, 3>& A, std::array<uvgvpcc_enc::Vector3<double>, 3>& Q,
                 std::array<uvgvpcc_enc::Vector3<double>, 3>& D) {
    const size_t maxsteps = p_->normalComputationMaxDiagonalStep;
    uvgvpcc_enc::Vector3<double> o;
    uvgvpcc_enc::Vector3<double> m;
    std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
    std::array<double, 4> jr{0.0, 0.0, 0.0, 0.0};
    std::array<uvgvpcc_enc::Vector3<double>, 3> AQ;

    for (size_t i = 0; i < maxsteps; ++i) {
        // quat to matrix
        const double sqx = q[0] * q[0];
        const double sqy = q[1] * q[1];
        const double sqz = q[2] * q[2];
        const double sqw = q[3] * q[3];
        Q[0][0] = (sqx - sqy - sqz + sqw);
        Q[1][1] = (-sqx + sqy - sqz + sqw);
        Q[2][2] = (-sqx - sqy + sqz + sqw);
        double tmp1 = q[0] * q[1];
        double tmp2 = q[2] * q[3];
        Q[1][0] = 2.0 * (tmp1 + tmp2);
        Q[0][1] = 2.0 * (tmp1 - tmp2);
        tmp1 = q[0] * q[2];
        tmp2 = q[1] * q[3];
        Q[2][0] = 2.0 * (tmp1 - tmp2);
        Q[0][2] = 2.0 * (tmp1 + tmp2);
        tmp1 = q[1] * q[2];
        tmp2 = q[0] * q[3];
        Q[2][1] = 2.0 * (tmp1 + tmp2);
        Q[1][2] = 2.0 * (tmp1 - tmp2);

        // AQ = A * Q;
        AQ[0][0] = Q[0][0] * A[0][0] + Q[1][0] * A[0][1] + Q[2][0] * A[0][2];
        AQ[0][1] = Q[0][1] * A[0][0] + Q[1][1] * A[0][1] + Q[2][1] * A[0][2];
        AQ[0][2] = Q[0][2] * A[0][0] + Q[1][2] * A[0][1] + Q[2][2] * A[0][2];
        AQ[1][0] = Q[0][0] * A[0][1] + Q[1][0] * A[1][1] + Q[2][0] * A[1][2];
        AQ[1][1] = Q[0][1] * A[0][1] + Q[1][1] * A[1][1] + Q[2][1] * A[1][2];
        AQ[1][2] = Q[0][2] * A[0][1] + Q[1][2] * A[1][1] + Q[2][2] * A[1][2];
        AQ[2][0] = Q[0][0] * A[0][2] + Q[1][0] * A[1][2] + Q[2][0] * A[2][2];
        AQ[2][1] = Q[0][1] * A[0][2] + Q[1][1] * A[1][2] + Q[2][1] * A[2][2];
        AQ[2][2] = Q[0][2] * A[0][2] + Q[1][2] * A[1][2] + Q[2][2] * A[2][2];

        // D  = Q.transpose() * AQ;
        D[0][0] = AQ[0][0] * Q[0][0] + AQ[1][0] * Q[1][0] + AQ[2][0] * Q[2][0];
        D[0][1] = AQ[0][0] * Q[0][1] + AQ[1][0] * Q[1][1] + AQ[2][0] * Q[2][1];
        D[0][2] = AQ[0][0] * Q[0][2] + AQ[1][0] * Q[1][2] + AQ[2][0] * Q[2][2];
        D[1][0] = AQ[0][1] * Q[0][0] + AQ[1][1] * Q[1][0] + AQ[2][1] * Q[2][0];
        D[1][1] = AQ[0][1] * Q[0][1] + AQ[1][1] * Q[1][1] + AQ[2][1] * Q[2][1];
        D[1][2] = AQ[0][1] * Q[0][2] + AQ[1][1] * Q[1][2] + AQ[2][1] * Q[2][2];
        D[2][0] = AQ[0][2] * Q[0][0] + AQ[1][2] * Q[1][0] + AQ[2][2] * Q[2][0];
        D[2][1] = AQ[0][2] * Q[0][1] + AQ[1][2] * Q[1][1] + AQ[2][2] * Q[2][1];
        D[2][2] = AQ[0][2] * Q[0][2] + AQ[1][2] * Q[1][2] + AQ[2][2] * Q[2][2];

        o[0] = D[1][2];
        o[1] = D[0][2];
        o[2] = D[0][1];
        m[0] = fabs(o[0]);
        m[1] = fabs(o[1]);
        m[2] = fabs(o[2]);

        // NOLINTNEXTLINE(readability-avoid-nested-conditional-operator)
        const int k0 = (m[0] > m[1] && m[0] > m[2]) ? 0 : (m[1] > m[2]) ? 1 : 2;  // index of largest element of offdiag
        const int k1 = (k0 + 1) % 3;
        const int k2 = (k0 + 2) % 3;
        if (o[k0] == 0.0) {
            break;  // diagonal already
        }
        double thet = (D[k2][k2] - D[k1][k1]) / (2.0 * o[k0]);
        const double sgn = (thet > 0.0) ? 1.0 : -1.0;
        thet *= sgn;                                                                       // make it positive
        const double t = sgn / (thet + ((thet < 1.E6) ? sqrt(thet * thet + 1.0) : thet));  // sign(T)/(|T|+sqrt(T^2+1))
        const double c = 1.0 / sqrt(t * t + 1.0);                                          //  c= 1/(t^2+1) , t=s/c
        if (c == 1.0) {
            break;  // no room for improvement - reached machine precision.
        }
        jr[k0] = sgn * sqrt((1.0 - c) / 2.0);  // using 1/2 angle identity sin(a/2)
                                               // = sqrt((1-cos(a))/2)
        jr[k0] *= -1.0;                        // since our quat-to-matrix convention was for v*M instead of M*v
        jr[3] = sqrt(1.0 - jr[k0] * jr[k0]);
        if (jr[3] == 1.0) {
            break;  // reached limits of floating point precision
        }
        q[0] = (q[3] * jr[0] + q[0] * jr[3] + q[1] * jr[2] - q[2] * jr[1]);
        q[1] = (q[3] * jr[1] - q[0] * jr[2] + q[1] * jr[3] + q[2] * jr[0]);
        q[2] = (q[3] * jr[2] + q[0] * jr[1] - q[1] * jr[0] + q[2] * jr[3]);
        q[3] = (q[3] * jr[3] - q[0] * jr[0] - q[1] * jr[1] - q[2] * jr[2]);
        const double mq = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= mq;
        q[1] /= mq;
        q[2] /= mq;
        q[3] /= mq;
    }
}
// NOLINTEND(cppcoreguidelines-init-variables)


void computeNormal(uvgvpcc_enc::Vector3<double>& normal, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                   const uvgvpcc_enc::Vector3<typeGeometryInput>& point, const std::vector<size_t>& pointNn, const size_t nnCount) {
    uvgvpcc_enc::Vector3<double> bary{static_cast<double>(point[0]), static_cast<double>(point[1]), static_cast<double>(point[2])};
    for (size_t i = 1; i < nnCount; ++i) {  // The first point return by the KNN is the query point. It is the initial value of bary.
        bary += pointsGeometry[pointNn[i]];
    }
    bary /= nnCount;

    std::array<uvgvpcc_enc::Vector3<double>, 3> covMat = {
        uvgvpcc_enc::Vector3<double>(0.0, 0.0, 0.0),
        uvgvpcc_enc::Vector3<double>(0.0, 0.0, 0.0),
        uvgvpcc_enc::Vector3<double>(0.0, 0.0, 0.0)
    };

    computeCovMat(covMat, bary, nnCount, pointNn, pointsGeometry);

    std::array<uvgvpcc_enc::Vector3<double>, 3> Q;
    std::array<uvgvpcc_enc::Vector3<double>, 3> D;
    diagonalize(covMat, Q, D);

    D[0][0] = fabs(D[0][0]);
    D[1][1] = fabs(D[1][1]);
    D[2][2] = fabs(D[2][2]);

    if (D[0][0] < D[1][1] && D[0][0] < D[2][2]) {
        normal[0] = Q[0][0];
        normal[1] = Q[1][0];
        normal[2] = Q[2][0];
    } else if (D[1][1] < D[2][2]) {
        normal[0] = Q[0][1];
        normal[1] = Q[1][1];
        normal[2] = Q[2][1];
    } else {
        normal[0] = Q[0][2];
        normal[1] = Q[1][2];
        normal[2] = Q[2][2];
    }
}

} // Anonymous namespace

namespace NormalComputation {

void computeNormals(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::vector<uvgvpcc_enc::Vector3<double>>& normals,
                    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                    const std::vector<std::vector<size_t>>& pointsNNList) {
    uvgvpcc_enc::Logger::log<uvgvpcc_enc::LogLevel::TRACE>("PATCH GENERATION",
                             "Compute normals of frame " + std::to_string(frame->frameId) + "\n");
    assert(p_->normalComputationKnnCount <= pointsGeometry.size());

    for (size_t pointIdx = 0; pointIdx < pointsGeometry.size(); ++pointIdx) {
        computeNormal(normals[pointIdx], pointsGeometry, pointsGeometry[pointIdx], pointsNNList[pointIdx], p_->normalComputationKnnCount);
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudNormalComputation(frame,pointsGeometry,normals);
    }
}


}  // namespace NormalComputation
