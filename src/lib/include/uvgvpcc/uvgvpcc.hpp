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

#include "uvgvpcc/threadqueue.hpp"
#include "../utils/utils.hpp"
#include "../utils/parameters.hpp"


/**
 * \file
 * libuvgvpcc API
 */

namespace uvgvpcc_enc {

extern const Parameters* p_; // Const pointer to a non-const Parameter struct instance in parameters.cpp

struct Patch {
    std::size_t patchIndex_;
    std::size_t patchPpi_;  // viewId   // projection plane of the patch isn't it ? // to do : make an enum

    std::size_t normalAxis_;     // x
    std::size_t tangentAxis_;    // y
    std::size_t bitangentAxis_;  // z

    bool projectionMode_;  // 0: related to the min depth value; 1: related to the max value

    std::size_t posU_;  // u1_ minU_       // tangential shift
    std::size_t posV_;  // v1_ minV_       // bitangential shift
    std::size_t posD_;  // d1_ minD_       // depth shift

    std::size_t sizeD_;  // size for depth (to do : is it the real patch thickness or the maximum possible thickness?)

    std::size_t widthInPixel_ = 0;   // size for U
    std::size_t heightInPixel_ = 0;  // size for V
    std::size_t widthInOccBlk_;      // sizeU0_     // size of occupancy map (n*occupancyMapResolution_)
    std::size_t heightInOccBlk_;     // sizeV0_     // size of occupancy map (n*occupancyMapResolution_)
    std::size_t omPosX_;             // u0_         // location in packed image (n*occupancyMapResolution_) // lf  posBlkU
    std::size_t omPosY_;             // v0_         // location in packed image (n*occupancyMapResolution_)

    bool axisSwap_;  // patch orientation    // in canvas atlas  (false default, true axis swap)x-tu

    std::vector<typeGeometryInput> depthL1_;  // depth value First layer // to do : geo type def !? Or not ?
    std::vector<typeGeometryInput> depthL2_;  // depth value Second layer
    std::vector<std::size_t>
        depthPCidxL1_;  // Index of the point in the PC for (Surface separation?) attribute retrieving during attribute map generation
    std::vector<std::size_t> depthPCidxL2_;

    std::size_t size2DXInPixel_;
    std::size_t size2DYInPixel_;

    std::vector<bool> patchOccupancy_;  // occupancy map (downscaled world)

    // inter packing //
    std::size_t area_ = 0;
    std::size_t referencePatchId_ =
        g_infinitenumber;  // Store the id of the best reference patch in the previous frame. Notice that even if the current patch is not
                           // matched, a reference patch id is still found. Then, this reference id will be used to check if the iou treshold
                           // is respected or not, then indicating if this patch is matched or not.
    std::size_t bestMatchIdx = INVALID_PATCH_INDEX;
    // Store not the id but the position in the list of patch of the best reference patch in the previous frame. Notice that even if
    // the current patch is not matched, a reference patch id is still found. Then, this reference id will be used to check if the
    // iou treshold is respected or not, then indicating if this patch is matched or not.
    bool isMatched_ = false;    // This patch match with a patch from the previous frame (the iou treshold is respected).
    bool isReference_ = false;  // This patch is a reference for a patch in the next frame.

    bool isLinkToAMegaPatch = false;
    std::size_t unionPatchReferenceIdx = INVALID_PATCH_INDEX;

    void setAxis(std::size_t normalAxis, std::size_t tangentAxis, std::size_t bitangentAxis, bool projectionMode) {
        normalAxis_ = normalAxis;
        tangentAxis_ = tangentAxis;
        bitangentAxis_ = bitangentAxis;
        projectionMode_ = projectionMode;  // to do : projection mode optimization per patch
    }

    void setPatchPpi(std::size_t patchPpi) {
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
        str << ", omPosX_=" << omPosX_;
        str << ", omPosY_=" << omPosY_;
        str << ", axisSwap=" << axisSwap_;
        str << ", size2DXInPixel=" << size2DXInPixel_;
        str << ", size2DYInPixel=" << size2DYInPixel_;
        return str.str();
    }
};

struct GOF;

// to do : check if when a member of a struct is resize, the whole struct can be copied/mode in another memory location
struct Frame {
    std::size_t frameId;      // aka relative index (0 if first encoded frame)
    std::size_t frameNumber;  // aka number from the input frame file name
    std::weak_ptr<GOF> gof;

    std::string pointCloudPath;

    std::size_t pointCount;
    std::vector<Vector3<typeGeometryInput>> pointsGeometry;
    std::vector<Vector3<uint8_t>> pointsAttribute;

    std::vector<Patch> patchList;
    std::vector<std::size_t> patchPartition;  // Associate a point index to the index of its patch

    std::string pointCloudBasePath;

    std::size_t occupancyMapHeight = 0;
    std::size_t mapsHeight = 0;  // Will be a gof parameter

    std::vector<uint8_t> occupancyMap;

    std::vector<uint8_t> geometryMapL1;  // first layer
    std::vector<uint8_t> geometryMapL2;  // second layer

    std::vector<uint8_t> attributeMapL1;  // Store the three channels continuously (all R, then all G, than all B)
    std::vector<uint8_t> attributeMapL2;

    void setInputBaseName() {
        // Extract the base name of the input file, useful when exporting intermediary files such as point clouds or maps
        std::string baseName;
        std::size_t found = pointCloudPath.find_last_of('/');
        if (found != std::string::npos) {
            baseName = pointCloudPath.substr(found + 1);
        } else {
            baseName = pointCloudPath;
        }
        found = baseName.find_last_of('_');
        if (found != std::string::npos) {
            baseName = baseName.substr(0, found - 1);
        }
        pointCloudBasePath = baseName;
    }

    Frame(const std::size_t& frameId, const std::size_t& frameNumber, const std::string& pointCloudPath)
        : frameId(frameId), frameNumber(frameNumber), pointCloudPath(pointCloudPath), pointCount(0) {
        setInputBaseName();
    }

    void printInfo() const;
};


struct GOF {
    std::vector<std::shared_ptr<Frame>> frames;
    std::size_t nbFrames;
    std::size_t gofId;

    std::size_t mapsHeight;
    std::size_t occupancyMapHeight;

    std::string baseNameOccupancy = "";
    std::string baseNameGeometry = "";
    std::string baseNameAttribute = "";

    std::vector<uint8_t> bitstreamOccupancy;
    std::vector<uint8_t> bitstreamGeometry;
    std::vector<uint8_t> bitstreamAttribute;

    void completeFileBaseNames(const Parameters* param) {
        const std::string gofIdStr = zeroPad(static_cast<int>(gofId), 3);
        const std::string nbFramesStr = zeroPad(static_cast<int>(nbFrames), 3);
        const std::string widthOMStr = std::to_string(param->mapWidth / param->occupancyMapResolution);
        const std::string heightOMStr = std::to_string(mapsHeight / param->occupancyMapResolution);
        const std::string widthStr = std::to_string(param->mapWidth);
        const std::string heightStr = std::to_string(mapsHeight);

        try {
            baseNameOccupancy = param->basenameOccupancyFiles;
            baseNameOccupancy.replace(baseNameOccupancy.find("{GOFID}"), 7, gofIdStr)
                .replace(baseNameOccupancy.find("{FRAMECOUNT}"), 12, nbFramesStr)
                .replace(baseNameOccupancy.find("{WIDTH}"), 7, widthOMStr)
                .replace(baseNameOccupancy.find("{HEIGHT}"), 8, heightOMStr);

            baseNameGeometry = param->basenameGeometryFiles;
            baseNameGeometry.replace(baseNameGeometry.find("{GOFID}"), 7, gofIdStr)
                .replace(baseNameGeometry.find("{FRAMECOUNT}"), 12, nbFramesStr)
                .replace(baseNameGeometry.find("{WIDTH}"), 7, widthStr)
                .replace(baseNameGeometry.find("{HEIGHT}"), 8, heightStr);

            baseNameAttribute = param->basenameAttributeFiles;
            baseNameAttribute.replace(baseNameAttribute.find("{GOFID}"), 7, gofIdStr)
                .replace(baseNameAttribute.find("{FRAMECOUNT}"), 12, nbFramesStr)
                .replace(baseNameAttribute.find("{WIDTH}"), 7, widthStr)
                .replace(baseNameAttribute.find("{HEIGHT}"), 8, heightStr);
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "\n# Catch an exception while completing the file base names :\n# " + std::string(e.what()) +
                "\n# basenameOccupancyBlank : " + param->basenameOccupancyFiles + "\n# basenameOccupancy      : " + baseNameOccupancy +
                "\n# basenameGeometryBlank  : " + param->basenameGeometryFiles + "\n# basenameGeometry       : " + baseNameGeometry +
                "\n# basenameAttributeBlank : " + param->basenameAttributeFiles + "\n# basenameAttribute      : " + baseNameAttribute + "\n");
        }
    }
};


namespace API {

struct v3c_chunk {
    std::size_t len = 0;           // Length of data in buffer
    std::unique_ptr<char[]> data;  // Actual data (char type can be used to describe a byte. No need for uint8_t or unsigned char types.)
    std::vector<std::size_t> v3c_unit_sizes = {};

    v3c_chunk() = default;
    v3c_chunk(std::size_t len, std::unique_ptr<char[]> data) : len(len), data(std::move(data)) {}
};
/* A V3C unit stream is composed of only V3C units without parsing information in the bitstream itself.
    The parsing information is here given separately. */
struct v3c_unit_stream {
    std::size_t v3c_unit_size_precision_bytes = 0;
    std::queue<v3c_chunk> v3c_chunks = {};
    std::counting_semaphore<> available_chunks{0};
    std::mutex io_mutex;  // Locks production and consumption in the v3c_chunks queue
};

void initializeEncoder();
void setParameter(const std::string& parameterName,const std::string& parameterValue);
void encodeFrame(std::shared_ptr<Frame> frame, v3c_unit_stream* output);
void emptyFrameQueue();
void stopEncoder();

}  // namespace API

};  // namespace uvgvpcc_enc