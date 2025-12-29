/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024, Tampere University, ITU/ISO/IEC, project contributors
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

// slicingComputation.cpp - Implements PPI assignment for uvgVPCCenc using slicing

#include "slicingComputation.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ppiSegmenter.hpp"
#include "robin_hood.h"
#include "utils/fileExport.hpp"
#include "utils/parameters.hpp"
#include "utils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;
using namespace std;

namespace slicingComputation {

enum class Axis : uint8_t { X, Y, Z };

constexpr std::array<size_t, 2> axisX = {1, 2};
constexpr std::array<size_t, 2> axisY = {0, 2};
constexpr std::array<size_t, 2> axisZ = {1, 0};

// NOLINTBEGIN(misc-use-anonymous-namespace)
static robin_hood::unordered_map<size_t, std::array<std::vector<std::vector<size_t>>, 3>>
    exportIntermediateSubslices;   // For intermediate files exportation only
static std::mutex exportMapMutex;  // For intermediate files exportation only
// NOLINTEND(misc-use-anonymous-namespace)

namespace {

template <const std::array<size_t, 2>& axis>
constexpr size_t AxisIndex() {
    if constexpr (axis == axisX) {
        return 0;
    }
    if constexpr (axis == axisY) {
        return 1;
    }
    return 2;
}

// Method for matrix coefficient rotation
template <std::size_t N>  // Matrix size
constexpr std::array<std::array<int, N>, N> rotate90CCW(const std::array<std::array<int, N>, N>& mat) {
    std::array<std::array<int, N>, N> result{};
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            result[i][j] = mat[j][N - 1 - i];
        }
    }
    return result;
}

/******************************************************************************************************************/
/*---------                    Base Matrices for Reading Order in Neighbor Search                         --------*/
/******************************************************************************************************************/
constexpr size_t BS = 0;  // Blind spot positions : those positions are not checked
constexpr std::array<std::array<std::array<int, 7>, 7>, 6> baseMatricesHolesDist5 = {{{{{BS, BS, 19, 13, 14, BS, BS},
                                                                                        {BS, 20, 12, 7, 8, 11, BS},
                                                                                        {23, 18, 6, 5, 2, 4, 10},
                                                                                        {22, 17, 15, BS, 1, 3, 9},
                                                                                        {26, 21, 16, BS, BS, BS, BS},
                                                                                        {BS, 27, 25, 24, BS, BS, BS},
                                                                                        {BS, BS, 29, 28, BS, BS, BS}}},

                                                                                      {{{BS, BS, 14, 8, 9, BS, BS},
                                                                                        {BS, 15, 7, 4, 5, 6, BS},
                                                                                        {18, 13, 3, 2, 1, BS, BS},
                                                                                        {17, 12, 10, BS, BS, BS, BS},
                                                                                        {22, 16, 11, 19, BS, BS, BS},
                                                                                        {BS, 23, 21, 20, 24, 28, BS},
                                                                                        {BS, BS, 26, 25, 27, BS, BS}}},

                                                                                      {{{BS, BS, 15, 9, 10, BS, BS},
                                                                                        {BS, 16, 8, 4, 6, 7, BS},
                                                                                        {19, 14, 3, 2, 1, 5, BS},
                                                                                        {18, 13, 11, BS, BS, BS, BS},
                                                                                        {23, 17, 12, 20, BS, BS, BS},
                                                                                        {BS, 24, 22, 21, BS, BS, BS},
                                                                                        {BS, BS, 27, 26, 28, BS, BS}}},

                                                                                      {{{BS, BS, 16, 10, 11, BS, BS},
                                                                                        {BS, 17, 9, 4, 6, 8, BS},
                                                                                        {20, 15, 3, 2, 1, 5, 7},
                                                                                        {19, 14, 12, BS, BS, BS, BS},
                                                                                        {24, 18, 13, 21, BS, BS, BS},
                                                                                        {BS, 25, 23, 22, BS, BS, BS},
                                                                                        {BS, BS, 27, 26, BS, BS, BS}}},

                                                                                      {{{BS, BS, 13, 7, 8, BS, BS},
                                                                                        {BS, 14, 6, 4, 5, BS, BS},
                                                                                        {17, 12, 2, 1, BS, BS, BS},
                                                                                        {16, 11, 3, BS, BS, BS, BS},
                                                                                        {21, 15, 9, 10, 18, BS, BS},
                                                                                        {BS, 22, 20, 19, 23, 27, BS},
                                                                                        {BS, BS, 25, 24, 26, BS, BS}}},

                                                                                      {{{BS, BS, 8, 6, 7, BS, BS},
                                                                                        {BS, 12, 5, 4, BS, BS, BS},
                                                                                        {15, 9, 2, 1, BS, BS, BS},
                                                                                        {14, 11, 3, BS, BS, BS, BS},
                                                                                        {20, 13, 10, 16, 17, 25, BS},
                                                                                        {BS, 21, 19, 18, 22, 26, BS},
                                                                                        {BS, BS, 24, 23, 27, BS, BS}}}}};

template <size_t Nm, size_t Ns>  // Nm : Number of matrices | Ns : Matrix size
constexpr auto generateMatricesHoleDist5() {
    std::array<std::array<std::array<int, Ns>, Ns>, 4 * Nm> result{};
    int k = 0;
    for (const auto& matrix : baseMatricesHolesDist5) {
        result[k] = matrix;
        result[k + 1] = rotate90CCW(result[k]);
        result[k + 2] = rotate90CCW(result[k + 1]);
        result[k + 3] = rotate90CCW(result[k + 2]);
        k = k + 4;
    }
    return result;
}

constexpr auto bestCandidateOrderedListHolesDist5 = generateMatricesHoleDist5<6, 7>();

struct Vector2D {
    int x, y;
    bool operator==(const Vector2D& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Vector2D& other) const { return x != other.x || y != other.y; }
};

struct Point2D {
    int x, y;
    bool operator==(const Point2D& other) const { return x == other.x && y == other.y; }
    Point2D operator+(const Point2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Point2D& other) const { return {x - other.x, y - other.y}; }
};

constexpr int SHIFT_NA = std::numeric_limits<int>::max();  // Non valid shifts : those adjacent points are not checked

constexpr auto generateOrderedAdjacentPointSearch2DDist5() {
    std::array<std::array<Point2D, 49>, 24> result = {};

    // Fill with default value
    for (auto& matrix : result) {
        for (auto& shift : matrix) {
            shift = {SHIFT_NA, SHIFT_NA};
        }
    }

    for (size_t k = 0; k < 24; ++k) {
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                const size_t scoreToOrderedIndex = bestCandidateOrderedListHolesDist5[k][i][j];
                if (scoreToOrderedIndex != BS) {
                    // Order the shifts according to the score
                    result[k][scoreToOrderedIndex - 1] = {3 - i, j - 3};
                }
            }
        }
    }
    return result;
}
constexpr auto orderedAdjacentPointSearch2DDist5 = generateOrderedAdjacentPointSearch2DDist5();

// Store the number of 'valid' neighbor point to be searched, that is, the number of element within the list of
// orderedAdjacentPointSearch2DDist5 before reaching the first 'non-valid' point {0,0} the LUT is {{dx1, dy1}, {dx2, dy2}, ..., {dxn, dyn},
// {0, 0}, ..., {0, 0}} so, if a shift is the vector {0,0}, we have tested the whole list
constexpr std::array<size_t, 24> generateOrderedAdjacentPointSearch2DDist5Count() {
    std::array<size_t, 24> result{};
    for (size_t k = 0; k < result.size(); ++k) {
        const auto& points = orderedAdjacentPointSearch2DDist5[k];
        size_t idx = 0;
        for (; idx < points.size(); ++idx) {
            if (points[idx].x == SHIFT_NA && points[idx].y == SHIFT_NA) {
                break;
            }
        }
        result[k] = idx;
    }
    return result;
}
constexpr auto orderedAdjacentPointSearch2DDist5Count = generateOrderedAdjacentPointSearch2DDist5Count();

// Store numbers from 0 to 23 that are associated with one of the 24 matrices storing the neighboring scores.
// Notice that NA values are never reached
constexpr size_t NA = 48;  // Arbitrary impossible value (7x7-1)
constexpr std::array<std::array<size_t, 7>, 7> LookUpVectorSetHoleDist5 = {{{NA, NA, 12, 0, 23, NA, NA},
                                                                            {NA, 4, 8, 0, 19, 7, NA},
                                                                            {20, 16, 4, 0, 7, 11, 15},
                                                                            {1, 1, 1, NA, 3, 3, 3},
                                                                            {13, 9, 5, 2, 6, 18, 22},
                                                                            {NA, 5, 17, 2, 10, 6, NA},
                                                                            {NA, NA, 21, 2, 14, NA, NA}}};

// Distances LookUpTable
constexpr std::array<std::array<size_t, 7>, 7> distancesLUT = {{{8, 7, 6, 5, 6, 7, 8},
                                                                {7, 4, 3, 2, 3, 4, 7},
                                                                {6, 3, 1, 1, 1, 3, 6},
                                                                {5, 2, 1, 0, 1, 2, 5},
                                                                {6, 3, 1, 1, 1, 3, 6},
                                                                {7, 4, 3, 2, 3, 4, 7},
                                                                {8, 7, 6, 5, 6, 7, 8}}};

void intermediateFileSlicingFrameInit(const size_t& frameId) {
    const std::lock_guard<std::mutex> lock(exportMapMutex);
    auto& arr = exportIntermediateSubslices[frameId];
    for (auto& vec : arr) vec.clear();
}

inline size_t getPos1D(const int& x, const int& y) { return x + (y << p_->geoBitDepthVoxelized); }
inline size_t getPos1D(const Point2D& pt) { return pt.x + (pt.y << p_->geoBitDepthVoxelized); }

constexpr size_t POS1D_NA = std::numeric_limits<size_t>::max();
constexpr size_t INDEX_NA = std::numeric_limits<size_t>::max();

template <const std::array<size_t, 2>& axis>
struct MapSearch {
    robin_hood::unordered_flat_map<size_t, size_t> pos1DToIndexSlicePointsNotInASubsliceYet;
    std::vector<size_t> currentSubsliceChildPos1D;

    MapSearch(const std::vector<size_t>& slice, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry) {
        pos1DToIndexSlicePointsNotInASubsliceYet.reserve(slice.size());
        currentSubsliceChildPos1D.reserve(slice.size());

        for (size_t pointIndexSlice = 0; pointIndexSlice < slice.size(); ++pointIndexSlice) {
            const size_t pos1D = getPos1D(pointsGeometry[slice[pointIndexSlice]][axis[0]], pointsGeometry[slice[pointIndexSlice]][axis[1]]);
            pos1DToIndexSlicePointsNotInASubsliceYet[pos1D] = pointIndexSlice;
        }
    }

    size_t findNeighborIndexSlice(const size_t& adjPos1D) {
        const auto itNew = pos1DToIndexSlicePointsNotInASubsliceYet.find(adjPos1D);
        if (itNew != pos1DToIndexSlicePointsNotInASubsliceYet.end()) {
            return itNew->second;
        }
        return INDEX_NA;
    }

    void addSubsliceChild(const size_t& pos1D) { currentSubsliceChildPos1D.emplace_back(pos1D); }

    void endOfSubslice() {
        // End sublice -> remove parents and childs of the current subslice from the slice
        // Removal form the pos1DToIndexSlicePointsNotInASubsliceYet map of the childs linked to the points in the subslice
        // These childs will never be freed (they are not in the slice Map to be found again by the FindNeighbors method)
        for (const auto& childPos1D : currentSubsliceChildPos1D) {
            this->erase(childPos1D);
        }
        currentSubsliceChildPos1D.clear();
    }

    void erase(const size_t& pos1D) { pos1DToIndexSlicePointsNotInASubsliceYet.erase(pos1D); }
};

template <const std::array<size_t, 2>& axis>
inline bool findNSetNextPoint(size_t& bestCandidateIndexSlice, size_t& distanceBestCandidate,
                              robin_hood::unordered_map<size_t, size_t>& childToParentAxis, const size_t& currentPointIndexPG,
                              const std::vector<size_t>& slice, std::vector<bool>& isInASubslice, MapSearch<axis>& mapSearch,
                              const Point2D& currentPoint2D, const Vector2D& previousVector) {
    const size_t bitDepth = 1U << p_->geoBitDepthVoxelized;

    // select index of the right adjacent point list --> Need vector V in the function
    // Offset to transform [-3, 3] → [0, 6]
    const size_t indexVectorTable = LookUpVectorSetHoleDist5[3 - previousVector.x][previousVector.y + 3];
    assert(indexVectorTable != NA);

    const auto& shiftList = orderedAdjacentPointSearch2DDist5[indexVectorTable];
    const size_t adjacentPointCount = orderedAdjacentPointSearch2DDist5Count[indexVectorTable];

    size_t shiftIndex = 0;
    // The first loop finds the best candidate, that is the first neighbor point to exist (the shift list is ordered so that the first found
    // neighboring point is the best candidate).
    for (; shiftIndex < adjacentPointCount; ++shiftIndex) {
        const auto& shift = shiftList[shiftIndex];
        const Point2D adjPoint2D = currentPoint2D + shift;
        if (adjPoint2D.x < 0 || adjPoint2D.x > bitDepth || adjPoint2D.y < 0 || adjPoint2D.y > bitDepth) continue;

        const size_t adjPos1D = getPos1D(adjPoint2D);

        const size_t neighborIndexSlice = mapSearch.findNeighborIndexSlice(adjPos1D);
        if (neighborIndexSlice != INDEX_NA) {
            bestCandidateIndexSlice = neighborIndexSlice;
            distanceBestCandidate = distancesLUT[3 - shift.x][shift.y + 3];
            mapSearch.erase(adjPos1D);
            break;
        }
    }

    if (shiftIndex == adjacentPointCount) {
        // No best candidate was found, so the current point has no neighbor.
        return false;
    }

    ++shiftIndex;

    // The second loop add all found points (after the best candidate) whose distance is less than the best candidate distance + 1, and make
    // them childs.
    const size_t distanceBestCandidateExtended = distanceBestCandidate + 1;

    for (; shiftIndex < adjacentPointCount; ++shiftIndex) {
        const auto& shift = shiftList[shiftIndex];

        // The best candidate has been found, so compute the distance of the potential next point, and consider it as a new child only if the
        // distance is < or = to the distance of the best candidate
        if (distancesLUT[3 - shift.x][shift.y + 3] > distanceBestCandidateExtended) continue;

        const Point2D adjPoint2D = currentPoint2D + shift;
        if (adjPoint2D.x < 0 || adjPoint2D.x > bitDepth || adjPoint2D.y < 0 || adjPoint2D.y > bitDepth) continue;

        const size_t adjPos1D = getPos1D(adjPoint2D);
        const size_t neighborIndexSlice = mapSearch.findNeighborIndexSlice(adjPos1D);
        if (neighborIndexSlice != INDEX_NA) {
            if (isInASubslice[neighborIndexSlice]) continue;
            isInASubslice[neighborIndexSlice] = true;
            const size_t neighborIndexPG = slice[neighborIndexSlice];
            childToParentAxis[neighborIndexPG] = currentPointIndexPG;
            mapSearch.addSubsliceChild(adjPos1D);
        }
    }

    return true;  // The current point has neighbors
}

// /* VectorMethod function is calculating the dot product of vect and the projection planes' normals according to the study axis */
// normalVectorIntoPPI
constexpr std::array<std::array<PPI, 4>, 3> normalVectorIntoPPI = {{// Trigonometric sense
                                                                    // axisX
                                                                    {{PPI::ppi4, PPI::ppi5, PPI::ppi1, PPI::ppi2}},
                                                                    // axisY
                                                                    {{PPI::ppi3, PPI::ppi5, PPI::ppi0, PPI::ppi2}},
                                                                    // axisZ
                                                                    {{PPI::ppi4, PPI::ppi3, PPI::ppi1, PPI::ppi0}}}};

inline std::ptrdiff_t idx(size_t i) { return static_cast<std::ptrdiff_t>(i); }

inline void filling(std::vector<PPI>& parentOrderedPPIs, const size_t& lowIndex, const size_t& upIndex, const PPI& lowPPI, const PPI& upPPI) {
    if (lowPPI == upPPI) {
        std::fill(parentOrderedPPIs.begin() + idx(lowIndex) + 1, parentOrderedPPIs.begin() + idx(upIndex), lowPPI);
        return;
    }

    const size_t len = upIndex - lowIndex - 1;
    const size_t lenHalf = (len / 2) + 1;
    std::fill(parentOrderedPPIs.begin() + idx(lowIndex) + 1, parentOrderedPPIs.begin() + idx(lowIndex) + 1 + idx(lenHalf), lowPPI);
    std::fill(parentOrderedPPIs.begin() + idx(lowIndex) + 1 + idx(lenHalf), parentOrderedPPIs.begin() + idx(upIndex), upPPI);
}

inline void slicePPISmoothing(std::vector<PPI>& parentOrderedPPIs) {
    size_t lowIndex = {};
    bool insideSegment = false;
    for (size_t orderedSliceIndex = 1; orderedSliceIndex < parentOrderedPPIs.size(); ++orderedSliceIndex) {
        if (!insideSegment && parentOrderedPPIs[orderedSliceIndex] == PPI::ppiBlank &&
            parentOrderedPPIs[orderedSliceIndex - 1] != PPI::ppiBlank) {
            lowIndex = orderedSliceIndex - 1;
            insideSegment = true;
        } else if (insideSegment && parentOrderedPPIs[orderedSliceIndex] != PPI::ppiBlank) {
            const PPI lowPPI = parentOrderedPPIs[lowIndex];
            const PPI upPPI = parentOrderedPPIs[orderedSliceIndex];
            filling(parentOrderedPPIs, lowIndex, orderedSliceIndex, lowPPI, upPPI);
            insideSegment = false;
        }
    }
}

inline bool compareBestCandidateAndStartingScores(const Point2D& bestCandidatePoint2D, const Point2D& currentPoint2D,
                                                  const Vector2D& previousVector, const Vector2D& startingVector) {
    if (previousVector.x == -startingVector.x && previousVector.y == -startingVector.y) {
        // If the vector to the starting point is the oposite of the PreviousPoint to StartingPoint vector, then the current point is not an
        // ending point, this is not the end of the subslice
        return false;
    }

    // Choose grid index, offset to transform [-3, 3] → [0, 6]
    const size_t gridSearchIndex = LookUpVectorSetHoleDist5[3 - previousVector.x][previousVector.y + 3];
    assert(gridSearchIndex != NA);

    // temporary, just for matrix search offset
    const size_t scoreStartingPoint = bestCandidateOrderedListHolesDist5[gridSearchIndex][3 - startingVector.x][startingVector.y + 3];
    if (scoreStartingPoint == BS) {
        // The starting point is in a blind spot. The starting point is then a better choice than the best candidate. This is the end of the
        // subslice.
        return true;
    }

    const Vector2D bestCandidateVector = bestCandidatePoint2D - currentPoint2D;
    const size_t scoreBestCandidate =
        bestCandidateOrderedListHolesDist5[gridSearchIndex][3 - bestCandidateVector.x][bestCandidateVector.y + 3];
    return scoreStartingPoint <= scoreBestCandidate;
    // true : The starting point has the same score (or a better one) that the best candidate. The starting point become the 'best option',
    // and so this is the end of the subslice. false : The starting point is part of the neighbor points of the current points, but its score
    // is worse than the score of the best candidate. The best candidate will then become the next point. This is not the end of the subslice.
}

template <const std::array<size_t, 2>& axis>
inline void sortSlice(const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, std::vector<size_t>& slice) {
    std::sort(slice.begin(), slice.end(), [&](const size_t& a, const size_t& b) {
        // if (pointsGeometry[a][axis[0]] == pointsGeometry[b][axis[0]]) {
        // Allow consistency
        //     return pointsGeometry[a][axis[1]] < pointsGeometry[b][axis[1]];
        // }
        // with a runtime parameter, it slows down the execution. We would need compilation time parameters
        return pointsGeometry[a][axis[0]] < pointsGeometry[b][axis[0]];
    });
}

template <const std::array<size_t, 2>& axis>
inline bool isEndOfSubslice(const size_t& distanceBestCandidate, const Point2D& currentPoint2D, const Vector2D& previousVector,
                            const Point2D& bestCandidatePoint2D, const Vector2D& startingVector) {
    // Distance calculation between the starting point and the current point
    size_t starting_point_dist = {};
    if (abs(startingVector.x) == 1 && abs(startingVector.y) == 1) {
        starting_point_dist = 1;
        // All points within the 3x3 square have the same score
    } else {
        starting_point_dist = startingVector.x * startingVector.x + startingVector.y * startingVector.y;
    }

    assert(starting_point_dist != 0);  // starting point is current point (start of subslice) (already checked before)

    // If the starting point is closer than the candidate point, there is no need to compute the angle, and this is the end of the subslice
    if (starting_point_dist < distanceBestCandidate) return true;

    // The best candidate is better than the starting point, it is not the end of the subslice
    if (starting_point_dist > distanceBestCandidate) return false;

    // starting_point_dist == distanceBestCandidate : need to compute the actual scores
    return compareBestCandidateAndStartingScores(bestCandidatePoint2D, currentPoint2D, previousVector, startingVector);
}

template <const std::array<size_t, 2>& axis>
inline PPI getPreviousPPI(const Vector2D& previousVector) {
    // Compute dot products
    const std::array<int, 4> dotProducts = {previousVector.y, -previousVector.x, -previousVector.y, previousVector.x};
    // Find index of maximum dot product
    const size_t max_idx = std::distance(dotProducts.begin(), std::max_element(dotProducts.begin(), dotProducts.end()));
    return normalVectorIntoPPI[AxisIndex<axis>()][max_idx];
}

// Weaving of a single subslice
template <const std::array<size_t, 2>& axis>
void subsliceWeaving(
    const size_t subsliceStartingPointIndexPG,  // Point geometry (global shared indexing) index of the starting point in the subslice
    const Point2D& startingPoint2D,
    const std::vector<size_t>& slice,  // Slice containing the point indices in PG
    const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, robin_hood::unordered_map<size_t, size_t>& childToParentAxis,
    std::vector<bool>& isInASubslice,           // Flags to track which slice points are already in a subslice
    std::vector<size_t>& parentOrderedIndexPG,  // Ordered list of parents (global indices)
    std::vector<PPI>& parentOrderedPPIs,        // Ordered list of PPIs corresponding to parents
    std::vector<size_t>& subsliceOrderedIndexPGIntermediaryExportation, MapSearch<axis>& mapSearch, const size_t& frameId) {
    size_t currentPointIndexPG = subsliceStartingPointIndexPG;
    Point2D currentPoint2D = startingPoint2D;
    Vector2D startingVector = {0, 0};  // start - current
    Vector2D previousVector = {1, 0};  // previousVector (also indicate which search matrix to use)
    bool isStartPoint = true;

    // Point after point, create a subslice
    while (true) {
        /*--- Find the best candidate and its childs ---*/
        size_t distanceBestCandidate = {};
        size_t bestCandidateIndexSlice = {};
        const bool hasNeighbor =
            findNSetNextPoint<axis>(bestCandidateIndexSlice, distanceBestCandidate, childToParentAxis, currentPointIndexPG, slice,
                                    isInASubslice, mapSearch, currentPoint2D, previousVector);
        if (!hasNeighbor) {
            break;  // No new neighbors: end of subslice
        }

        const size_t bestCandidateIndexPG = slice[bestCandidateIndexSlice];
        const Point2D bestCandidatePoint2D = {pointsGeometry[bestCandidateIndexPG][axis[0]], pointsGeometry[bestCandidateIndexPG][axis[1]]};

        /*--- Check if subslice should close (return to starting point) ---*/
        if (!isStartPoint) {
            const bool closeSubslice =
                isEndOfSubslice<axis>(distanceBestCandidate, currentPoint2D, previousVector, bestCandidatePoint2D, startingVector);
            if (closeSubslice) {
                if (!isInASubslice[bestCandidateIndexSlice]) {
                    // Mark the best candidate as child of the current point
                    childToParentAxis[bestCandidateIndexPG] = currentPointIndexPG;
                    isInASubslice[bestCandidateIndexSlice] = true;
                }
                break;  // Subslice ends by looping back to its starting point
            }
        }

        /*--- Update current point with best candidate ---*/

        // Update direction vector (between current and next)
        previousVector = {bestCandidatePoint2D.x - currentPoint2D.x, bestCandidatePoint2D.y - currentPoint2D.y};
        if (isStartPoint) {
            parentOrderedPPIs.emplace_back(getPreviousPPI<axis>(previousVector));
        } else {
            if (abs(previousVector.x) == abs(previousVector.y)) {
                // Diagonal points will get a ppi during ppi filling.
                parentOrderedPPIs.emplace_back(PPI::ppiBlank);
            } else {
                parentOrderedPPIs.emplace_back(getPreviousPPI<axis>(previousVector));
            }
        }
        currentPoint2D = bestCandidatePoint2D;
        currentPointIndexPG = bestCandidateIndexPG;
        startingVector = {startingPoint2D.x - currentPoint2D.x, startingPoint2D.y - currentPoint2D.y};

        if (isInASubslice[bestCandidateIndexSlice]) {
            // The best candidate point has been marked as a child previously in this subslice. Remove this child/parent link.
            childToParentAxis.erase(bestCandidateIndexPG);
        } else {
            isInASubslice[bestCandidateIndexSlice] = true;
        }

        if (p_->exportIntermediateFiles) {
            subsliceOrderedIndexPGIntermediaryExportation.emplace_back(bestCandidateIndexPG);
        }
        parentOrderedIndexPG.emplace_back(bestCandidateIndexPG);
        isStartPoint = false;
    }

    ////////////////////
    /// end subslice ///
    ////////////////////

    if (p_->exportIntermediateFiles) {
        if (axis == axisX) exportIntermediateSubslices[frameId][0].emplace_back(subsliceOrderedIndexPGIntermediaryExportation);
        if (axis == axisY) exportIntermediateSubslices[frameId][1].emplace_back(subsliceOrderedIndexPGIntermediaryExportation);
        if (axis == axisZ) exportIntermediateSubslices[frameId][2].emplace_back(subsliceOrderedIndexPGIntermediaryExportation);
    }

    // PPI attribution for the last point in the subslice
    if (isStartPoint) {  // Subslice made of only one point
        parentOrderedPPIs.emplace_back(PPI::ppi0);
    } else {
        if (parentOrderedPPIs.back() == PPI::ppiBlank) {
            // If last point has a blank ppi, assign a ppi (create a bound for the filling ppi process)
            const Vector2D previousVectorTemp = {startingPoint2D.x - currentPoint2D.x, startingPoint2D.y - currentPoint2D.y};
            parentOrderedPPIs.emplace_back(getPreviousPPI<axis>(previousVectorTemp));
        } else {
            parentOrderedPPIs.emplace_back(parentOrderedPPIs.back());
        }
    }

    mapSearch.endOfSubslice();
}

template <const std::array<size_t, 2>& axis>
void sliceWeaving(const std::vector<size_t>& slice, const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                  robin_hood::unordered_map<size_t, size_t>& childToParentAxis, std::vector<PPI>& pointPPIsAxis, const size_t& frameId) {
    const size_t sliceSize = slice.size();
    int startingPointIndexSlice = -1;

    // Flags to track if a slice point is already assigned to a subslice
    std::vector<bool> isInASubslice(slice.size(), false);

    std::vector<PPI> parentOrderedPPIs;
    parentOrderedPPIs.reserve(sliceSize);

    std::vector<size_t> parentOrderedIndexPG;
    parentOrderedIndexPG.reserve(sliceSize);

    // Stores indices of the current subslice for intermediate export
    std::vector<size_t> subsliceOrderedIndexPGIntermediaryExportation;

    MapSearch<axis> mapSearch(slice, pointsGeometry);

    // Slice weaving: process each subslice until there is no remaining point in the slice
    while (true) {
        // Find first point not already in a subslice (slice points are ordered along X axis)
        auto itStart = std::find(isInASubslice.begin() + startingPointIndexSlice + 1, isInASubslice.end(), false);
        if (itStart == isInASubslice.end()) {
            break;  // All slice points are assigned, end of slice weaving
        }
        startingPointIndexSlice = static_cast<int>(std::distance(isInASubslice.begin(), itStart));

        // Initialize new subslice
        const size_t startingPointIndexPG = slice[startingPointIndexSlice];  // Starting point of the subslice
        const Point2D startingPoint2D = {pointsGeometry[startingPointIndexPG][axis[0]], pointsGeometry[startingPointIndexPG][axis[1]]};
        const size_t startingPos1D = getPos1D(startingPoint2D);
        mapSearch.erase(startingPos1D);
        parentOrderedIndexPG.emplace_back(startingPointIndexPG);
        isInASubslice[startingPointIndexSlice] = true;

        if (p_->exportIntermediateFiles) {
            subsliceOrderedIndexPGIntermediaryExportation.clear();
            subsliceOrderedIndexPGIntermediaryExportation.emplace_back(startingPointIndexPG);
        }

        subsliceWeaving<axis>(startingPointIndexPG, startingPoint2D, slice, pointsGeometry, childToParentAxis, isInASubslice,
                              parentOrderedIndexPG, parentOrderedPPIs, subsliceOrderedIndexPGIntermediaryExportation, mapSearch, frameId);
    }

    /////////////////////
    ///// end slice /////
    /////////////////////

    // Smooth PPI and final PPI attribution for this axis
    slicePPISmoothing(parentOrderedPPIs);
    for (size_t orderedSliceIndex = 0; orderedSliceIndex < parentOrderedPPIs.size(); ++orderedSliceIndex) {
        pointPPIsAxis[parentOrderedIndexPG[orderedSliceIndex]] = parentOrderedPPIs[orderedSliceIndex];
    }
}

void childPPIAttribution(const robin_hood::unordered_map<size_t, size_t>& childToParentX,
                         const robin_hood::unordered_map<size_t, size_t>& childToParentY,
                         const robin_hood::unordered_map<size_t, size_t>& childToParentZ, std::vector<size_t>& pointPPIs) {
    for (size_t ptIndexPG = 0; ptIndexPG < pointPPIs.size(); ++ptIndexPG) {
        if (pointPPIs[ptIndexPG] < 6) continue;  // Already has a PPI

        auto itY = childToParentY.find(ptIndexPG);
        if (itY != childToParentY.end()) {
            const size_t ppiParentY = pointPPIs[itY->second];
            if (ppiParentY < 6) {
                pointPPIs[ptIndexPG] = ppiParentY;
                continue;
            }
        }

        auto itX = childToParentX.find(ptIndexPG);
        if (itX != childToParentX.end()) {
            const size_t ppiParentX = pointPPIs[itY->second];
            if (ppiParentX < 6) {
                pointPPIs[ptIndexPG] = ppiParentX;
                continue;
            }
        }

        auto itZ = childToParentZ.find(ptIndexPG);
        if (itZ != childToParentZ.end()) {
            const size_t ppiParentZ = pointPPIs[itY->second];
            if (ppiParentZ < 6) {
                pointPPIs[ptIndexPG] = ppiParentZ;
                continue;
            }
        }

        assert(false);  // No valid parent found (impossible)
    }
}

void createSlices(const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                  std::vector<std::optional<std::vector<size_t>>>& levelToSliceX,
                  std::vector<std::optional<std::vector<size_t>>>& levelToSliceY,
                  std::vector<std::optional<std::vector<size_t>>>& levelToSliceZ) {
    const size_t nbMaxSlices = (1U << p_->geoBitDepthVoxelized);
    const size_t estNbPointsPerSlice = nbMaxSlices * 2;

    for (size_t ptIndexPG = 0; ptIndexPG < pointsGeometry.size(); ++ptIndexPG) {
        const auto& point = pointsGeometry[ptIndexPG];

        // X axis
        auto& xSlice = levelToSliceX[point[0]];
        if (!xSlice.has_value()) {
            xSlice = std::vector<size_t>();
            xSlice->reserve(estNbPointsPerSlice);
        }
        xSlice->push_back(ptIndexPG);

        // Y axis
        auto& ySlice = levelToSliceY[point[1]];
        if (!ySlice.has_value()) {
            ySlice = std::vector<size_t>();
            ySlice->reserve(estNbPointsPerSlice);
        }
        ySlice->push_back(ptIndexPG);

        // Z axis
        auto& zSlice = levelToSliceZ[point[2]];
        if (!zSlice.has_value()) {
            zSlice = std::vector<size_t>();
            zSlice->reserve(estNbPointsPerSlice);
        }
        zSlice->push_back(ptIndexPG);
    }
}

template <const std::array<size_t, 2>& axis>
void createTempPointCloudForSlicingExportation(const std::shared_ptr<Frame>& frame,
                                               const std::vector<Vector3<typeGeometryInput>>& pointsGeometry) {
    std::vector<Vector3<uint8_t>> attributes(pointsGeometry.size());
    typeGeometryInput currentAxisLevel = {};
    typeGeometryInput previousAxisLevel = std::numeric_limits<typeGeometryInput>::max();
    size_t countSubslicePerSlice = 0;

    size_t axisIndex = {};
    if constexpr (axis == axisX) axisIndex = 0;
    if constexpr (axis == axisY) axisIndex = 1;
    if constexpr (axis == axisZ) axisIndex = 2;

    for (const auto& subslice : exportIntermediateSubslices[frame->frameId][axisIndex]) {
        assert(subslice.size() > 0);
        // Take the first point of the subslice to determine the axis level.
        currentAxisLevel = pointsGeometry[subslice[0]][axisIndex];
        if (currentAxisLevel == previousAxisLevel) {
            ++countSubslicePerSlice;
        } else {
            countSubslicePerSlice = 0;
        }
        previousAxisLevel = currentAxisLevel;
        attributes[subslice[0]] = FileExport::ppiColors[2];
        if (subslice.size() == 1) {
            attributes[subslice[0]] = FileExport::ppiColors[6];
            continue;
        }

        attributes[subslice[1]] = FileExport::ppiColors[1];
        for (size_t indexSubslice = 2; indexSubslice < subslice.size() - 1; ++indexSubslice) {
            const size_t ptIndexPG = subslice[indexSubslice];
            const uint8_t r = static_cast<uint8_t>((1.0 + static_cast<float>(countSubslicePerSlice) / 3.0) *
                                                   static_cast<float>(FileExport::ppiColors[3][0])) %
                              255;
            const uint8_t g = static_cast<uint8_t>((1.0 + static_cast<float>(countSubslicePerSlice) / 3.0) *
                                                   static_cast<float>(FileExport::ppiColors[3][1])) %
                              255;
            const uint8_t b = static_cast<uint8_t>((1.0 + static_cast<float>(countSubslicePerSlice) / 3.0) *
                                                   static_cast<float>(FileExport::ppiColors[3][2])) %
                              255;
            attributes[ptIndexPG] = {r, g, b};
        }
        attributes[subslice[subslice.size() - 1]] = FileExport::ppiColors[4];
    }

    std::string axisStr = {};
    if constexpr (axis == axisX) axisStr = "X";
    if constexpr (axis == axisY) axisStr = "Y";
    if constexpr (axis == axisZ) axisStr = "Z";
    FileExport::exportPointCloudSubslices(frame, pointsGeometry, attributes, axisStr);
}

template <const std::array<size_t, 2>& axis>
inline void axisSlicesWeaving(std::vector<std::optional<std::vector<size_t>>>& levelToSlice,
                              const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                              robin_hood::unordered_map<size_t, size_t>& childToParent, std::vector<PPI>& pointPPIsAxis,
                              const size_t& frameId) {
    childToParent.reserve(pointsGeometry.size());
    for (auto& sliceOpt : levelToSlice) {
        if (sliceOpt.has_value()) {
            auto& slice = sliceOpt.value();
            sortSlice<axis>(pointsGeometry, slice);
            sliceWeaving<axis>(slice, pointsGeometry, childToParent, pointPPIsAxis, frameId);
        }
    }
}

// TODO(lf): use PPI type everywhere

// Set the normal for weighted points
inline Vector3<double> ppiToNormal(const PPI& ppi) {
    switch (ppi) {
        case PPI::ppi0:
            return {1.0, 0.0, 0.0};  // normal[0] =  1.0; break;
        case PPI::ppi1:
            return {0.0, 1.0, 0.0};  // normal[1] =  1.0; break;
        case PPI::ppi2:
            return {0.0, 0.0, 1.0};  // normal[2] =  1.0; break;
        case PPI::ppi3:
            return {-1.0, 0.0, 0.0};  // normal[0] = -1.0; break;
        case PPI::ppi4:
            return {0.0, -1.0, 0.0};  // normal[1] = -1.0; break;
        case PPI::ppi5:
            return {0.0, 0.0, -1.0};  // normal[2] = -1.0; break;
        default:
            assert(false);
    }
    assert(false);
    return {0.0, 0.0, 0.0};
}

inline size_t getParentPpi(const PPI& ppiX, const PPI& ppiY, const PPI& ppiZ, const size_t& nAttributions, Vector3<double>& normal) {
    if (nAttributions == 1) return UNDEFINED_PARENT_PPI;

    // If two axes agree → choose that PPI and mark point as weighted (unity normal)
    if (ppiX == ppiY || ppiZ == ppiY) {
        normal = ppiToNormal(ppiY);
        return static_cast<size_t>(ppiY);
    }
    if (ppiX == ppiZ) {
        normal = ppiToNormal(ppiX);
        return static_cast<size_t>(ppiX);
    }
    if (nAttributions == 3) {
        // All axis ppi differ → pick Y-axis PPI, but point is not weighted (normal stays (0,0,0))
        return static_cast<size_t>(ppiY);
    }

    // nAttributions==2 but the two temporary axis ppi differ → undefined parent
    return UNDEFINED_PARENT_PPI;
}

inline size_t getUndefinedParentPpi(const std::vector<size_t>& pointPPIs, const PPI& ppiX, const PPI& ppiY, const PPI& ppiZ,
                                    const robin_hood::unordered_map<size_t, size_t>& childToParentX,
                                    const robin_hood::unordered_map<size_t, size_t>& childToParentY,
                                    const robin_hood::unordered_map<size_t, size_t>& childToParentZ, const size_t& idx) {
    // An undefined parent is a point with ambiguous or incomplete PPI attribution.
    // Strategy:
    //  1) Try to inherit a valid PPI from its own parent along missing axes.
    //  2) If no valid inherited PPI, fall back to available axis PPIs.
    //  Priority order: Y > X > Z.

    // Inherit from parent along Y-axis if this axis has no PPI
    if (ppiY == PPI::notAssigned) {
        const size_t ppiParentY = pointPPIs[childToParentY.at(idx)];
        if (ppiParentY < 6) {
            return ppiParentY;
        }
    }

    // Inherit from parent along X-axis
    if (ppiX == PPI::notAssigned) {
        const size_t ppiParentX = pointPPIs[childToParentX.at(idx)];
        if (ppiParentX < 6) {
            return ppiParentX;
        }
    }

    // Inherit from parent along Z-axis
    if (ppiZ == PPI::notAssigned) {
        const size_t ppiParentZ = pointPPIs[childToParentZ.at(idx)];
        if (ppiParentZ < 6) {
            return ppiParentZ;
        }
    }

    // If no inheritance is possible → fallback to already assigned axis PPIs.
    // Axis priority: Y first, then X, then Z.
    if (ppiY != PPI::notAssigned) return static_cast<size_t>(ppiY);
    if (ppiX != PPI::notAssigned) return static_cast<size_t>(ppiX);
    return static_cast<size_t>(ppiZ);
}

// TODO(lf): in the end handle all memory swap etc...
void finalPPIAttributionFastPreset(const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                                   const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                   const std::vector<PPI>& pointPPIsX, const std::vector<PPI>& pointPPIsY, const std::vector<PPI>& pointPPIsZ,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentX,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentY,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentZ, std::vector<size_t>& pointPPIs) {
    const size_t nbPoints = pointsGeometry.size();

    // A parent point is a point with at least one temporary PPI
    // (assigned during axisSlicesWeaving on X, Y, or Z axis).
    // A child point is one that will later inherit its PPI from its parent.
    //
    // Parent points can be:
    //  - Weighted: at least two axes agree → unity normal (e.g., (1,0,0))
    //  - Non-weighted: no strong agreement or undefined PPI → normal (0,0,0)
    // With fast preset, child points don't have a normal as they don't pass through refine segmentation.
    //
    // In the fast preset:
    // 1) Only parent points are refined (need for temporary data structure).
    // 2) After refinement, children inherit PPI from their parents.

    std::vector<size_t> parentPointsPPIs(nbPoints);
    std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>> parentPointsGeometry(nbPoints);
    std::vector<uvgvpcc_enc::Vector3<double>> parentPointsNormal(nbPoints, {{0.0, 0.0, 0.0}});
    std::vector<size_t> parentPointsIndexInPG(nbPoints);

    size_t sizeParentSublist = 0;  // For refine segmentation parent sublist data structure creation
    for (int ptIndexPG = 0; ptIndexPG < nbPoints; ++ptIndexPG) {
        const PPI ppiX = pointPPIsX[ptIndexPG];
        const PPI ppiY = pointPPIsY[ptIndexPG];
        const PPI ppiZ = pointPPIsZ[ptIndexPG];

        size_t nAttributions = 0;
        if (ppiX != PPI::notAssigned) ++nAttributions;
        if (ppiY != PPI::notAssigned) ++nAttributions;
        if (ppiZ != PPI::notAssigned) ++nAttributions;

        // Skip non-parent points (no temporary PPI from any axis).
        if (nAttributions == 0) continue;

        // Assign PPI and determine parent normal (weighted or non-weighted).
        pointPPIs[ptIndexPG] = getParentPpi(ppiX, ppiY, ppiZ, nAttributions, parentPointsNormal[sizeParentSublist]);

        if (pointPPIs[ptIndexPG] == UNDEFINED_PARENT_PPI) {
            // Mark as undefined parent, will be resolved later.
            continue;
        }

        // Store defined parent in the sublist for refine segmentation.
        pointPPIs[ptIndexPG] = pointPPIs[ptIndexPG];
        parentPointsPPIs[sizeParentSublist] = pointPPIs[ptIndexPG];
        parentPointsIndexInPG[sizeParentSublist] = ptIndexPG;
        parentPointsGeometry[sizeParentSublist] = pointsGeometry[ptIndexPG];
        ++sizeParentSublist;
    }

    // Handle undefined parents (always considered non-weighted → normal (0,0,0)).
    for (size_t ptIndexPG = 0; ptIndexPG < nbPoints; ++ptIndexPG) {
        if (pointPPIs[ptIndexPG] != UNDEFINED_PARENT_PPI) continue;
        const size_t parentPpi = getUndefinedParentPpi(pointPPIs, pointPPIsX[ptIndexPG], pointPPIsY[ptIndexPG], pointPPIsZ[ptIndexPG],
                                                       childToParentX, childToParentY, childToParentZ, ptIndexPG);

        // Store undefined parent in the sublist for refine segmentation.
        pointPPIs[ptIndexPG] = parentPpi;
        parentPointsPPIs[sizeParentSublist] = parentPpi;
        parentPointsIndexInPG[sizeParentSublist] = ptIndexPG;
        parentPointsGeometry[sizeParentSublist] = pointsGeometry[ptIndexPG];
        parentPointsNormal[sizeParentSublist] = {0.0, 0.0, 0.0};
        ++sizeParentSublist;
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudPPIAttributionSlicing(frame, pointsGeometry, pointPPIs);
    }

    // Refine segmentation only on the parent sublist.
    parentPointsGeometry.resize(sizeParentSublist);
    parentPointsNormal.resize(sizeParentSublist);
    parentPointsPPIs.resize(sizeParentSublist);
    PPISegmenter ppiSegmenter(parentPointsGeometry, parentPointsNormal);
    ppiSegmenter.refineSegmentation(frame, parentPointsPPIs, frame->frameId);

    // Copy refined PPI values back to the full list of points.
    for (int ptIndexSublist = 0; ptIndexSublist < sizeParentSublist; ++ptIndexSublist) {
        const size_t parentPointIndexPG = parentPointsIndexInPG[ptIndexSublist];
        pointPPIs[parentPointIndexPG] = parentPointsPPIs[ptIndexSublist];
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudRefineSegmentation(frame, parentPointsGeometry, parentPointsPPIs);
    }

    // Finally, propagate PPI from refined parents to their children.
    childPPIAttribution(childToParentX, childToParentY, childToParentZ, pointPPIs);
}

void finalPPIAttributionSlowPreset(const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                                   const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry,
                                   const std::vector<PPI>& pointPPIsX, const std::vector<PPI>& pointPPIsY, const std::vector<PPI>& pointPPIsZ,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentX,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentY,
                                   const robin_hood::unordered_map<size_t, size_t>& childToParentZ, std::vector<size_t>& pointPPIs) {
    const size_t nbPoints = pointsGeometry.size();

    // A parent point is a point with at least one temporary PPI
    // (assigned during axisSlicesWeaving on X, Y, or Z axis).
    // A child point is one that will inherit its PPI from a parent.
    //
    // Parent points can be:
    //  - Weighted: at least two axes agree → unity normal (e.g., (1,0,0))
    //  - Non-weighted: no strong agreement or undefined PPI → normal (0,0,0)
    // Child points always have normal (0,0,0).

    std::vector<uvgvpcc_enc::Vector3<double>> pointsNormal(nbPoints);
    for (int ptIndexPG = 0; ptIndexPG < nbPoints; ++ptIndexPG) {
        const PPI ppiX = pointPPIsX[ptIndexPG];
        const PPI ppiY = pointPPIsY[ptIndexPG];
        const PPI ppiZ = pointPPIsZ[ptIndexPG];

        size_t nAttributions = 0;
        if (ppiX != PPI::notAssigned) ++nAttributions;
        if (ppiY != PPI::notAssigned) ++nAttributions;
        if (ppiZ != PPI::notAssigned) ++nAttributions;

        // Skip non-parent points (no temporary PPI from any axis).
        if (nAttributions == 0) continue;

        // Assign parent PPI and compute its normal (weighted or non-weighted).
        pointPPIs[ptIndexPG] = getParentPpi(ppiX, ppiY, ppiZ, nAttributions, pointsNormal[ptIndexPG]);
    }

    // Handle undefined parent points (always considered non-weighted → normal (0,0,0)):
    for (size_t ptIndexPG = 0; ptIndexPG < nbPoints; ++ptIndexPG) {
        if (pointPPIs[ptIndexPG] != UNDEFINED_PARENT_PPI) continue;
        pointPPIs[ptIndexPG] = getUndefinedParentPpi(pointPPIs, pointPPIsX[ptIndexPG], pointPPIsY[ptIndexPG], pointPPIsZ[ptIndexPG],
                                                     childToParentX, childToParentY, childToParentZ, ptIndexPG);
    }

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudPPIAttributionSlicing(frame, pointsGeometry, pointPPIs);
    }

    // Slow preset pipeline:
    // 1) First propagate PPI from parents to children
    // 2) Then refine segmentation on all points
    childPPIAttribution(childToParentX, childToParentY, childToParentZ, pointPPIs);
    PPISegmenter ppiSegmenter(pointsGeometry, pointsNormal);
    ppiSegmenter.refineSegmentation(frame, pointPPIs, frame->frameId);

    if (p_->exportIntermediateFiles) {
        FileExport::exportPointCloudRefineSegmentation(frame, pointsGeometry, pointPPIs);
    }
}

}  // anonymous namespace

void ppiAssignationSlicing(const std::shared_ptr<uvgvpcc_enc::Frame>& frame,
                           const std::vector<uvgvpcc_enc::Vector3<typeGeometryInput>>& pointsGeometry, std::vector<size_t>& pointPPIs) {
    // Create the 2D slices for each axis.
    const size_t nbMaxSlices = (1U << p_->geoBitDepthVoxelized);
    std::vector<std::optional<std::vector<size_t>>> levelToSliceX(nbMaxSlices);
    std::vector<std::optional<std::vector<size_t>>> levelToSliceY(nbMaxSlices);
    std::vector<std::optional<std::vector<size_t>>> levelToSliceZ(nbMaxSlices);
    createSlices(pointsGeometry, levelToSliceX, levelToSliceY, levelToSliceZ);

    const size_t frameId = frame->frameId;
    // Slice weaving : for each axis, set the PPI of the parent point and associate child points with parent points.
    if (p_->exportIntermediateFiles) intermediateFileSlicingFrameInit(frameId);

    const size_t nbPoints = pointsGeometry.size();
    std::vector<PPI> pointPPIsX(nbPoints, PPI::notAssigned);
    std::vector<PPI> pointPPIsY(nbPoints, PPI::notAssigned);
    std::vector<PPI> pointPPIsZ(nbPoints, PPI::notAssigned);
    robin_hood::unordered_map<size_t, size_t> childToParentX;
    robin_hood::unordered_map<size_t, size_t> childToParentY;
    robin_hood::unordered_map<size_t, size_t> childToParentZ;
    axisSlicesWeaving<axisX>(levelToSliceX, pointsGeometry, childToParentX, pointPPIsX, frameId);
    axisSlicesWeaving<axisY>(levelToSliceY, pointsGeometry, childToParentY, pointPPIsY, frameId);
    axisSlicesWeaving<axisZ>(levelToSliceZ, pointsGeometry, childToParentZ, pointPPIsZ, frameId);
    if (p_->exportIntermediateFiles) {
        createTempPointCloudForSlicingExportation<axisX>(frame, pointsGeometry);
        createTempPointCloudForSlicingExportation<axisY>(frame, pointsGeometry);
        createTempPointCloudForSlicingExportation<axisZ>(frame, pointsGeometry);
        exportIntermediateSubslices.erase(frameId);
    }

    // Final PPI attribution
    // TODO(lf): use an enum for presetName
    if (p_->presetName == "fast") {
        finalPPIAttributionFastPreset(frame, pointsGeometry, pointPPIsX, pointPPIsY, pointPPIsZ, childToParentX, childToParentY,
                                      childToParentZ, pointPPIs);
    } else if (p_->presetName == "slow") {
        finalPPIAttributionSlowPreset(frame, pointsGeometry, pointPPIsX, pointPPIsY, pointPPIsZ, childToParentX, childToParentY,
                                      childToParentZ, pointPPIs);
    } else {
        assert(false);
    }
}

}  // namespace slicingComputation