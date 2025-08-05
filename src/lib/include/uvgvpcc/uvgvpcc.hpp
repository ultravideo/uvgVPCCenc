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

#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <semaphore>
#include <sstream>
#include <string>
#include <vector>

#include "../utils/utils.hpp"

/// \file Main file of the uvgVPCCenc library that defines the main structures (GOF, frame, patch) and the API.

// TODO(lf): why in include/uvgvpcc/ ? What about the related .cpp Why not "include/" only ?

namespace uvgvpcc_enc {

// A patch is a 3D object. Though, the word 'patch' can refer to the "2D version" of this patch
struct Patch {
    size_t patchIndex_;
    size_t patchPpi_;  // viewId

    size_t normalAxis_;     // x
    size_t tangentAxis_;    // y
    size_t bitangentAxis_;  // z

    size_t posU_;  // u1_ minU_       // tangential shift
    size_t posV_;  // v1_ minV_       // bitangential shift
    size_t posD_;  // d1_ minD_       // depth shift

    bool projectionMode_;  // 0: related to the min depth value; 1: related to the max value

    size_t sizeD_;  // while posD_ is the minimum 'depth', sizeD_ store the maximum depth. TODO(lf): check if it is usefull

    std::vector<uint8_t> patchOccupancyMap_;  // patch occupancy map (boolean vector)

    size_t widthInPixel_ = 0;   // size for U  // width of the patch occupancy map (in pixels)
    size_t heightInPixel_ = 0;  // size for V  // height of the patch occupancy map (in pixels)

    size_t widthInOccBlk_;  // sizeU0_     // width of the patch occupancy map within the down-scaled frame occupancy map (in DS occupancy map
                            // blocks).
    size_t heightInOccBlk_;  // sizeV0_     // height of the patch occupancy map within the down-scaled frame occupancy map (in DS occupancy
                             // map blocks).

    size_t omDSPosX_;  // u0_         // location in down-scaled occupancy map  // lf  posBlkU
    size_t omDSPosY_;  // v0_         // location in down-scaled occupancy map

    bool axisSwap_;  // patch orientation    // in canvas atlas  (false default, true axis swap)

    std::vector<typeGeometryInput> depthL1_;  // depth value First layer // TODO(lf): Using the geo type here might lead to issue?
    std::vector<typeGeometryInput> depthL2_;  // depth value Second layer

    // Index of the point in the PC for attribute retrieving during attribute map generation TODO(lf): use for Surface separation?
    std::vector<size_t> depthPCidxL1_;
    std::vector<size_t> depthPCidxL2_;

    // inter packing //
    size_t area_ = 0;

    size_t referencePatchId_ = g_infinitenumber;
    // Store the id of the best reference patch in the previous frame. Notice that even if the current patch is not
    // matched, a reference patch id is still found. Then, this reference id will be used to check if the iou treshold
    // is respected or not, then indicating if this patch is matched or not.

    size_t bestMatchIdx = INVALID_PATCH_INDEX;
    // Store not the id but the position in the list of patch of the best reference patch in the previous frame. Notice that even if
    // the current patch is not matched, a reference patch id is still found. Then, this reference id will be used to check if the
    // iou treshold is respected or not, then indicating if this patch is matched or not.
    bool isMatched_ = false;  // This patch match with a patch from the previous frame (the iou treshold is respected).
    bool isLinkToAMegaPatch = false;
    size_t unionPatchReferenceIdx = INVALID_PATCH_INDEX;

    void setAxis(size_t normalAxis, size_t tangentAxis, size_t bitangentAxis, bool projectionMode) {
        normalAxis_ = normalAxis;
        tangentAxis_ = tangentAxis;
        bitangentAxis_ = bitangentAxis;
        projectionMode_ = projectionMode;  // TODO(lf): projection mode optimization per patch
    }

    inline void setPatchPpiAndAxis(size_t patchPpi) {
        patchPpi_ = patchPpi;
        // now set the other variables according to the viewId
        switch (patchPpi_) {
            case 0:
                setAxis(0, 2, 1, 0);
                break;
            case 1:
                setAxis(1, 2, 0, 0);
                break;
            case 2:
                setAxis(2, 0, 1, 0);
                break;
            case 3:
                setAxis(0, 2, 1, 1);
                break;
            case 4:
                setAxis(1, 2, 0, 1);
                break;
            case 5:
                setAxis(2, 0, 1, 1);
                break;
            default:
                throw std::runtime_error("ViewId (" + std::to_string(patchPpi) + ") not allowed... exiting");
                break;
        }
    }

    std::string toString() const {
        std::stringstream str;
        str << "patchIndex=" << patchIndex_;
        str << ", patchPpi=" << patchPpi_;
        str << ", normalAxis=" << normalAxis_;
        str << ", tangentAxis=" << tangentAxis_;
        str << ", bitangentAxis=" << bitangentAxis_;
        str << ", projectionMode=" << projectionMode_;
        str << ", minU=" << posU_;
        str << ", minV=" << posV_;
        str << ", minD=" << posD_;
        str << ", sizeD=" << sizeD_;
        str << ", sizeU=" << widthInPixel_;
        str << ", sizeV=" << heightInPixel_;
        str << ", sizeUom=" << widthInOccBlk_;
        str << ", sizeVom=" << heightInOccBlk_;
        str << ", omDSPosX_=" << omDSPosX_;
        str << ", omDSPosY_=" << omDSPosY_;
        str << ", axisSwap=" << axisSwap_;
        return str.str();
    }
};

struct GOF;

// TODO(lf): Avid using both constant sized and dynamic sized memory member within the same struct.
struct Frame {
    size_t frameId;      // aka relative index (0 if first encoded frame)
    size_t frameNumber;  // aka number from the input frame file name (TODO(lf): correct)
    std::weak_ptr<GOF> gof;
    std::shared_ptr<std::counting_semaphore<UINT16_MAX>> conccurentFrameSem;

    std::string pointCloudPath;

    size_t pointCount;
    std::vector<Vector3<typeGeometryInput>> pointsGeometry;
    std::vector<Vector3<uint8_t>> pointsAttribute;

    std::vector<Patch> patchList;

    size_t mapHeight = 0;  // TODO(lf): Will be a gof parameter ?
    size_t mapHeightDS = 0;

    std::vector<uint8_t> occupancyMap;    // (boolean vector)
    std::vector<uint8_t> occupancyMapDS;  // Down-scaled occupancy map of the frame (boolean vector)

    std::vector<uint8_t> geometryMapL1;  // first layer
    std::vector<uint8_t> geometryMapL2;  // second layer

    std::vector<uint8_t> attributeMapL1;  // Store the three channels continuously (all R, then all G, than all B)
    std::vector<uint8_t> attributeMapL2;

    Frame(const size_t& frameId, const size_t& frameNumber, const std::string& pointCloudPath)
        : frameId(frameId), frameNumber(frameNumber), pointCloudPath(pointCloudPath), pointCount(0) {}
    ~Frame() {
        if (conccurentFrameSem) {
            conccurentFrameSem->release();
        }
    };
    void printInfo() const;
};

struct GOF {
    std::vector<std::shared_ptr<Frame>> frames;
    size_t nbFrames;
    size_t gofId;

    size_t mapHeightGOF;
    size_t mapHeightDSGOF;

    std::vector<uint8_t> bitstreamOccupancy;
    std::vector<uint8_t> bitstreamGeometry;
    std::vector<uint8_t> bitstreamAttribute;
};

/// @brief API of the uvgVPCCenc library
namespace API {

/// @brief Bitstream writing miscellaneous
struct v3c_chunk {
    size_t len = 0;                // Length of data in buffer
    std::unique_ptr<char[]> data;  // Actual data (char type can be used to describe a byte. No need for uint8_t or unsigned char types.)
    std::vector<size_t> v3c_unit_sizes = {};

    v3c_chunk() = default;
    v3c_chunk(size_t len, std::unique_ptr<char[]> data) : len(len), data(std::move(data)) {}
};

// ht: A V3C unit stream is composed of only V3C units without parsing information in the bitstream itself. The parsing information is here
// given separately.
struct v3c_unit_stream {
    size_t v3c_unit_size_precision_bytes = 0;
    std::queue<v3c_chunk> v3c_chunks = {};
    std::counting_semaphore<> available_chunks{0};
    std::mutex io_mutex;  // Locks production and consumption in the v3c_chunks queue
};

void initializeEncoder();
void setParameter(const std::string& parameterName, const std::string& parameterValue);
void encodeFrame(std::shared_ptr<Frame>& frame, v3c_unit_stream* output);
void emptyFrameQueue();
void stopEncoder();

}  // namespace API

};  // namespace uvgvpcc_enc
