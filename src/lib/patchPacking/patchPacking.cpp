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

/// \file Entry point for the patch packing process. Assign a 2D location to each patch.


#include "patchPacking.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <span>
#include <string>
#include <vector>
#include <cassert>

#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"
#include "utils/utils.hpp"

using namespace uvgvpcc_enc;

PatchPacking::PatchPacking() = default;

inline bool PatchPacking::checkFitPatch(const size_t& patchPosX, const size_t& patchPosY, const size_t& patchWidth,
                                        const size_t& patchHeight, const size_t& mapHeight, const std::vector<uint8_t>& frameOccupancyMap) {
    // TODO(lf): deprecated comments

    // Iterate through the currrent patch occupancy. Weither a block is occupied or not in the patch occupancy map is not considered. The
    // whole patch bounding box need to fit in the occupancy map. (cf precedence in TMC2)

    // So, we check if a rectangle (the bounding box of the patch) can fit at the current location in the OM (posOMu and posOMv)
    // To create a space between patches, we increase the size of this rectangle in all directions by a certain amount, depending on
    // p_->spacePatchPacking. Notice that we don't increase the size of the rectangle in directions for which the patch overlap a border of
    // the map. Indeed, a space is needed between the patch but not between a patch and the map border.

    // To optimize this checking process, we first check all corners of this rectangle. Then we check the perimeter. Then all remaining OM
    // block.
    const size_t spacePatchPacking = p_->spacePatchPacking * p_->occupancyMapDSResolution;

    // Everything in this function is accoridng to the downscaled occupancy map. So the unit is the OM block size. (It means that, for
    // example, in this function, patchWidth corresponds to patch.widthInOccBlk_ and not patch.widthInPixel_) The bounding box of the patch is
    // characterized by its size (patchWidth, patchHeight) and its current position (patchPosX,patchPosY). This position will be validated or
    // not by this function. The rectangle is characterized by its size (areaWidth, areaHeight) and its position (areaPosX,areaPosY) To
    // iterate through the rectangle, we use areaX and areaY The corresponding coordinate on the occupancy map are mapX and mapY

    // TODO(lf): use accumulate function to compute sum of occupancy map block for big area that are likely to be empty, instead of for loop
    // with if condition TODO(lf): is there a risk for overflow with spacePatchPacking-1 or any other unsigned typed variable like this ?
    // spacePatchPacking can be 0

    const size_t mapWidth = p_->mapWidth;

    const size_t areaPosX = patchPosX - std::min(spacePatchPacking, patchPosX);  // Check if near left map border
    const size_t areaPosY = patchPosY - std::min(spacePatchPacking, patchPosY);  // Check if near top map border

    // Check top left corner of the area
    if (static_cast<bool>(frameOccupancyMap[areaPosX + areaPosY * mapWidth])) {
        return false;
    }

    const size_t areaWidth =
        patchWidth + std::min(spacePatchPacking, patchPosX) + std::min(spacePatchPacking, mapWidth - (patchPosX + patchWidth));

    // Check top right corner of the area
    if (static_cast<bool>(frameOccupancyMap[areaPosX + areaWidth - 1 + areaPosY * mapWidth])) {
        return false;
    }

    const size_t areaHeight =
        patchHeight + std::min(spacePatchPacking, patchPosY) + std::min(spacePatchPacking, mapHeight - (patchPosY + patchHeight));

    // Check bottom left corner of the area
    if (static_cast<bool>(frameOccupancyMap[areaPosX + (areaPosY + areaHeight - 1) * mapWidth])) {
        return false;
    }
    // Check bottom right corner of the area
    if (static_cast<bool>(frameOccupancyMap[areaPosX + areaWidth - 1 + (areaPosY + areaHeight - 1) * mapWidth])) {
        return false;
    }

    // Check the top and bottom perimeter section of the area
    for (size_t mapPos = areaPosX + 1 + areaPosY * mapWidth; mapPos < areaPosX + areaWidth - 1 + areaPosY * mapWidth;++mapPos) {
        if (static_cast<bool>(frameOccupancyMap[mapPos])) {
            return false;
        }
        if (static_cast<bool>(frameOccupancyMap[mapPos + (areaHeight - 1) * mapWidth])) {
            return false;
        }
    }

    // Check the left and right perimeter section of the area
    for (size_t areaY = 1; areaY < areaHeight - 1; ++areaY) {
        if (static_cast<bool>(frameOccupancyMap[areaPosX + (areaY + areaPosY) * mapWidth])) {
            return false;
        }
        if (static_cast<bool>(frameOccupancyMap[areaPosX + areaWidth - 1 + (areaY + areaPosY) * mapWidth])) {
            return false;
        }
    }

    // Check remaining pixels of the area // TODO(lf): should do mapY+=p_->resolutionDS and idem for mapX and everywhere else in the function (consider only DS blocks indirectly) (depends on the minimum size of a patch)
    for (size_t mapY = areaPosY + 1; mapY < areaPosY + areaHeight - 1; ++mapY) {
        for (size_t mapX = areaPosX + 1; mapX < areaPosX + areaWidth - 1; ++mapX) {
            const size_t mapPos = mapX + mapY * mapWidth;
            if (static_cast<bool>(frameOccupancyMap[mapPos])) {
                return false;
            }
        }
    }

    return true;
}

inline bool PatchPacking::checkLocation(const size_t& mapHeight, const size_t& posOMu, const size_t& posOMv, const size_t& patchWidth, const size_t& patchHeight,
                                        size_t& maxPatchHeight, uvgvpcc_enc::Patch& patch, const std::vector<uint8_t>& frameOccupancyMap) {

    const size_t heightBound = posOMv + patchHeight;
    const size_t widthBound = posOMu + patchWidth;

    if (heightBound > mapHeight || widthBound > p_->mapWidth) {
        // At this location (posOmU,posOmV), the patch bounding box overlap the occupancy map border
        return false;
    }    

    const bool locationFound = checkFitPatch(posOMu, posOMv, patchWidth, patchHeight, mapHeight, frameOccupancyMap);
    if (locationFound) {
        patch.omDSPosX_ = posOMu/p_->occupancyMapDSResolution;
        patch.omDSPosY_ = posOMv/p_->occupancyMapDSResolution;
        maxPatchHeight = std::max(maxPatchHeight, heightBound);
        return true;
    }

    return false;
}

bool PatchPacking::findPatchLocation(const size_t& mapHeight, size_t& maxPatchHeight,uvgvpcc_enc::Patch& patch, const std::vector<uint8_t>& frameOccupancyMap) {
    // Iterate over the occupancy map. For each position, check if the patch fit with the default orientation and with its axis swaped.
    bool locationFound = false;
    const size_t step = (1 + p_->spacePatchPacking)*p_->occupancyMapDSResolution;
    assert(patch.widthInOccBlk_ * p_->occupancyMapDSResolution == patch.widthInPixel_);
    for (size_t posOMv = 0; posOMv < mapHeight && !locationFound; posOMv += step ) {
        for (size_t posOMu = 0; posOMu < p_->mapWidth; posOMu += step) {
            locationFound = checkLocation(mapHeight, posOMu, posOMv,  patch.widthInPixel_,
                                          patch.heightInPixel_, maxPatchHeight, patch,frameOccupancyMap);
            if (locationFound) {
                patch.axisSwap_ = false;
                return true;
            }

            // Swap patch width and height
            locationFound = checkLocation(mapHeight, posOMu, posOMv, patch.heightInPixel_,
                                          patch.widthInPixel_, maxPatchHeight, patch, frameOccupancyMap);
            if (locationFound) {
                patch.axisSwap_ = true;
                return true;
            }
        }
    }
    return false;
}

void PatchPacking::allocateDefaultOccupancyMap(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, const size_t mapHeight) {
    frame->mapHeight = mapHeight; // TODO(lf): can be done in the Frame constructor? (bad idea to put all memory allocaion at the same time no, or I mean too early ?)
    frame->mapHeightDS = mapHeight / p_->occupancyMapDSResolution; // TODO(lf): can be done in the Frame constructor? (bad idea to put all memory allocaion at the same time no, or I mean too early ?)
    frame->occupancyMap.resize(p_->mapWidth * frame->mapHeight,0);

}

// TODO(lf): First test swap patch rotation mode if this minimize hypothetic resulting map height


// Patch placement and indirect occupancy map generation //
void PatchPacking::frameIntraPatchPacking(const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::span<uvgvpcc_enc::Patch>* patchListSpan) {
    if (!p_->interPatchPacking) {
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH PACKING",
                                 "Intra pack patches of frame " + std::to_string(frame->frameId) + ".\n");
    }

    // If the inter patch packing mode is deactivated, the intra patch packing is done over all frame patches. Thus, the patchListSpan
    // corresponds to the frame patch list. When the inter patch packing is activated, this intra packing function will be called only for the
    // remaining non-matched patch. The patchListSpan will then correspond to the a specific section of the frame patch list.
    const std::span<uvgvpcc_enc::Patch>& patchList = patchListSpan != nullptr ? *patchListSpan : frame->patchList;

    size_t mapHeightTemp = frame->mapHeight;
    size_t maxPatchHeight = 0; // Maximum height occupied by a patch

    // Iterate over all patches of the frame //
    for (auto& patch : patchList) {
        for (;;) {
            const bool locationFound = findPatchLocation(mapHeightTemp,maxPatchHeight, patch,frame->occupancyMap);
            if (locationFound) {
                break;
            }

            mapHeightTemp *= 2;
            frame->occupancyMap.resize(p_->mapWidth * mapHeightTemp);
        }

        // Update the occupancy map by adding the current patch at its found location //
        if (!patch.axisSwap_) {
            // Line by line, copy the 'patch DS occupancy map' into the 'frame DS occupancy map' at the previously found patch location //
            for (size_t patchY = 0; patchY < patch.heightInPixel_; ++patchY) {
                const size_t srcOffset = patchY * patch.widthInPixel_;
                const size_t dstOffset = (patch.omDSPosY_ * p_->occupancyMapDSResolution + patchY) * p_->mapWidth 
                                + patch.omDSPosX_ * p_->occupancyMapDSResolution;

                auto srcIt = patch.patchOccupancyMap_.begin();
                auto dstIt = frame->occupancyMap.begin();
                std::copy(
                    srcIt + static_cast<ptrdiff_t>(srcOffset),
                    srcIt + static_cast<ptrdiff_t>(srcOffset + patch.widthInPixel_),
                    dstIt + static_cast<ptrdiff_t>(dstOffset)
                );

            }

        } else {
            // As the patch axis are swaped, bulk copy (line by line) is not possible
            // The patch occupancy is read in a simple order (line by line)
            // The area of the occupancy map is written in an swapped way : colomn by colomn
            for(size_t patchX=0; patchX<patch.widthInPixel_; ++patchX) {
                for(size_t patchY=0; patchY<patch.heightInPixel_; ++patchY) {
                    frame->occupancyMap[patch.omDSPosX_*p_->occupancyMapDSResolution + patchY + (patchX+patch.omDSPosY_*p_->occupancyMapDSResolution) *p_->mapWidth ] = patch.patchOccupancyMap_[patchX + patchY * patch.widthInPixel_];
                }
            }
        }
    }





    frame->mapHeight = std::max(frame->mapHeight, maxPatchHeight);
    frame->mapHeightDS = frame->mapHeight / p_->occupancyMapDSResolution;
    
}

// Patch placement and indirect occupancy map generation using union patch information for the matched patch //
void PatchPacking::frameInterPatchPacking(const std::vector<uvgvpcc_enc::Patch>& unionPatches,
                                          const std::shared_ptr<uvgvpcc_enc::Frame>& frame, std::span<uvgvpcc_enc::Patch>* matchedPatchList) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "GLOBAL PATCH PACKING",
                             "Inter patch packing of the matched patches of frame " + std::to_string(frame->frameId) + ".\n");

    // Iterate over all patches of the frame that are matched with a union match //
    for (auto& patch : *matchedPatchList) {
        const auto& linkedMegaPatch = unionPatches[patch.unionPatchReferenceIdx];
        patch.omDSPosX_ = linkedMegaPatch.omDSPosX_;  // TODO(lf): might be nice to center the matched patch within the boundary of the union patch
        patch.omDSPosY_ = linkedMegaPatch.omDSPosY_;
        patch.axisSwap_ = linkedMegaPatch.axisSwap_;

        // TODO(ls) : code repetition with intra function (create a new function for writing patch?)

        // Update the occupancy map by adding the current patch at its found location //
        if (!patch.axisSwap_) {
            // Line by line, copy the patch occupancy into the occupancy map at the previously found location //
            for(size_t patchY=0; patchY<patch.heightInPixel_; ++patchY) {
                for(size_t patchX=0; patchX<patch.widthInPixel_; ++patchX) {
                    frame->occupancyMap[patch.omDSPosX_*p_->occupancyMapDSResolution + patchX + (patchY+patch.omDSPosY_*p_->occupancyMapDSResolution) * p_->mapWidth ] = patch.patchOccupancyMap_[patchX + patchY * patch.widthInPixel_];
                }
            }

        } else {
            // As the patch axis are swaped, bulk copy (line by line) is not possible
            // The patch occupancy is read in a simple order (line by line)
            // The area of the occupancy map is written in an swapped way : colomn by colomn
            for(size_t patchX=0; patchX<patch.widthInPixel_; ++patchX) {
                for(size_t patchY=0; patchY<patch.heightInPixel_; ++patchY) {
                    frame->occupancyMap[patch.omDSPosX_*p_->occupancyMapDSResolution + patchY + (patchX+patch.omDSPosY_*p_->occupancyMapDSResolution) * p_->mapWidth ] = patch.patchOccupancyMap_[patchX + patchY * patch.widthInPixel_];
                }
            }    

        }
    }
}

float PatchPacking::computeIoU(const uvgvpcc_enc::Patch& currentPatch, const uvgvpcc_enc::Patch& previousPatch) {
    // Compute the intersection of the space in the 3D world, from the point of view of the projection plan (both patch have the same
    // projection axis).

    // Calculate the edges of the current patch
    const size_t currentRight = currentPatch.posU_ + currentPatch.widthInPixel_;
    const size_t currentBottom = currentPatch.posV_ + currentPatch.heightInPixel_;

    // Calculate the edges of the previous patch
    const size_t previousRight = previousPatch.posU_ + previousPatch.widthInPixel_;
    const size_t previousBottom = previousPatch.posV_ + previousPatch.heightInPixel_;

    // Calculate the edges of the intersection rectangle
    const size_t intersectLeft = std::max(currentPatch.posU_, previousPatch.posU_);
    const size_t intersectRight = std::min(currentRight, previousRight);

    const size_t intersectTop = std::max(currentPatch.posV_, previousPatch.posV_);  // Y axis pointing to the bottom
    const size_t intersectBottom = std::min(currentBottom, previousBottom);         // Y axis pointing to the bottom

    // Calculate the width and height of the intersection rectangle
    const int intersectWidth = static_cast<int>(intersectRight) - static_cast<int>(intersectLeft);
    const int intersectHeight = static_cast<int>(intersectBottom) - static_cast<int>(intersectTop);  // Y axis pointing to the bottom

    if (intersectWidth <= 0 ||
        intersectHeight <= 0) {  // TODO(lf)early retun possible by dividing in vertical and horizaontl the cde (so two return 0.F)
        return 0.F;
    }

    // If width and height are positive, compute the intersection area.
    const size_t intersectionArea = static_cast<size_t>(intersectWidth) * static_cast<size_t>(intersectHeight);
    const size_t unionArea = currentPatch.area_ + previousPatch.area_ - intersectionArea;
    const float iou =
        static_cast<float>(intersectionArea) / static_cast<float>(unionArea);  // intersection over union // TODO(lf)do everything in float ?
    return iou;
}

void PatchPacking::patchMatchingBetweenTwoFrames(const std::shared_ptr<uvgvpcc_enc::Frame>& currentFrame,
                                                 const std::shared_ptr<uvgvpcc_enc::Frame>& previousFrame) {
    int id = 0;

    // main loop.
    for (auto& patch : previousFrame->patchList) {
        id++;
        if (patch.bestMatchIdx == uvgvpcc_enc::INVALID_PATCH_INDEX) {
            continue;
        }

        float maxIou = 0.0F;
        int bestIdx = -1;
        int cId = 0;
        for (auto& cpatch : currentFrame->patchList) {  // my comment : current patch
            if ((patch.patchPpi_ % 3 == cpatch.patchPpi_ % 3) && cpatch.bestMatchIdx == uvgvpcc_enc::INVALID_PATCH_INDEX) {
                // TODO(lf)flip normal of a patch to make them point toward the nearest projection plan (amoung the
                // two that are aligned on the projection axis) during the normal orientation process
                const float iou = computeIoU(patch, cpatch);
                if (iou > maxIou) {
                    maxIou = iou;
                    bestIdx = cId;
                }
            }
            cId++;
        }

        if (maxIou > p_->gpaTresholdIoU) {
            currentFrame->patchList[bestIdx].bestMatchIdx = id - 1;  // best previous frame patch index
        }
    }
}

void PatchPacking::gofPatchPacking(const std::shared_ptr<uvgvpcc_enc::GOF>& gof) {
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH PACKING",
                             "Inter pack patches of GOF " + std::to_string(gof->gofId) + ".\n");

    auto& firstFrame = gof->frames[0];
    if (gof->nbFrames == 1) {
        // No inter packing if the GOF is composed of a single frame
        // Basic intra packing method
        std::span<uvgvpcc_enc::Patch> patchList(firstFrame->patchList);
        allocateDefaultOccupancyMap(firstFrame,p_->minimumMapHeight);
        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH PACKING",
                                 "Intra pack patches of frame " + std::to_string(firstFrame->frameId) +
                                     " as it is the only frame within the GOF " + std::to_string(gof->gofId) + ".\n");
        frameIntraPatchPacking(firstFrame, &patchList);
        return;
    }

    // Set all patches of the first frame as matched
    // Assign a value different than uvgvpcc_enc::INVALID_PATCH_INDEX
    for (uvgvpcc_enc::Patch& patch : firstFrame->patchList) {
        patch.bestMatchIdx = 0;
    }

    // Iterate over all frames and compute the patch IoU between each of them to match the patches if possible
    for (size_t frameId = 1; frameId < gof->nbFrames; ++frameId) {
        patchMatchingBetweenTwoFrames(gof->frames[frameId], gof->frames[frameId - 1]);
    }

    // Iterate over the patches of the last frame of the gof and build the unionPatches. //
    std::vector<uvgvpcc_enc::Patch> unionPatches;
    unionPatches.reserve(64);  // High expectation

    // unionPatches are just used to find the best place for each matched patch. Like a blank patch. They do not
    // carry any other information than a bounding box, an area, an ID, the axis swap, and a position on the OM.
    auto& lastFrame = gof->frames[gof->nbFrames - 1];
    for (size_t lastPatchIdx = 0; lastPatchIdx < lastFrame->patchList.size(); ++lastPatchIdx) {
        if (lastFrame->patchList[lastPatchIdx].bestMatchIdx == uvgvpcc_enc::INVALID_PATCH_INDEX) {
            continue;
        }

        // This patch from the last frame of the GOF is matched. It means that there is a 'chain' of matched patches going through all frames.
        // A new union patch can be created. Add a new union patch
        unionPatches.emplace_back();
        const size_t unionPatchIdx = unionPatches.size() - 1;
        auto& unionPatch = unionPatches[unionPatchIdx];
        unionPatch.patchIndex_ = unionPatchIdx;

        // Iterate over all frames of the GOF, starting from the last, and update the union patch info by retrieving the one matched patch in
        // each frame. This looks like browsing a link list. to do, maybe storing pointer between patches is more efficient than the patch
        // index.
        size_t matchedPatchIdx = lastPatchIdx;
        for (auto frame = gof->frames.rbegin(); frame != gof->frames.rend(); ++frame) {
            auto* currentPatch = &(*frame)->patchList[matchedPatchIdx];
            currentPatch->isLinkToAMegaPatch = true;
            currentPatch->unionPatchReferenceIdx = unionPatchIdx;
            unionPatch.widthInOccBlk_ = std::max(unionPatch.widthInOccBlk_, currentPatch->widthInOccBlk_);
            unionPatch.heightInOccBlk_ = std::max(unionPatch.heightInOccBlk_, currentPatch->heightInOccBlk_);
            unionPatch.widthInPixel_ = unionPatch.widthInOccBlk_ * p_->occupancyMapDSResolution;
            unionPatch.heightInPixel_ = unionPatch.heightInOccBlk_ * p_->occupancyMapDSResolution;
            matchedPatchIdx = currentPatch->bestMatchIdx;
        }

        // Fill the patch occupancy map of the union patch //
        unionPatch.patchOccupancyMap_.resize(unionPatch.widthInPixel_ * unionPatch.heightInPixel_, 1);
    }

    const size_t nbUnionPatch = unionPatches.size();

    // Sort the union patches by size
    std::sort(unionPatches.begin(), unionPatches.end(), [](const uvgvpcc_enc::Patch& patchA, const uvgvpcc_enc::Patch& patchB) {
        return std::max(patchA.widthInOccBlk_, patchA.heightInOccBlk_) > std::max(patchB.widthInOccBlk_, patchB.heightInOccBlk_);
    });

    // Pack the union patches (use the first frame of the GOF as support) //
    // This is a "mock" step. A true patch packing will still be applied to the first frame.

    std::span<uvgvpcc_enc::Patch> unionPatchList(unionPatches);
    allocateDefaultOccupancyMap(firstFrame,p_->minimumMapHeight);
    uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH PACKING",
                             "Intra pack patches of the union patches of GOF " + std::to_string(gof->gofId) + ".\n");
    frameIntraPatchPacking(firstFrame, &unionPatchList);

    // TODO(lf): limitHeightOccupancyMap should be in occBlk not in pixels and should be GOF param

    // Update default occupancy map size for each frame //
    // This is usefull if during the patch packing of the union patches the height of the occupancy map has been modified (if it is greater
    // then the minimum height parameter). TODO(lf): this step might be useless once this param is at GOF level

    // TODO(lf): both in intra and inter patch packing, it is optimal to update the limitHeightOccupancyMap at each frame. Thus, some resizing of
    // the occupancy map might be done once. Only the occupancy map from frame before the max occupancy map height will have to be resized.
    // This will be easier once the limitHeightOccupancyMap is a GOF parameter.

    // Reset the occupancy map of the first frame //
    // Not done by default in the function frameIntraPatchPacking(...) as the resize operation do not write the '0' in the non
    // resized portion of the vector.
    std::fill(firstFrame->occupancyMap.begin(), firstFrame->occupancyMap.end(), 0);

    // Reorder the patches in each frame patch list so that the first ones are the matched ones (and that they respect the order of the union
    // patches). This is needed as this order is also the packing order, which is used by the decoder. TODO(lf) : verify
    for (auto& frame : gof->frames) {
        std::vector<uvgvpcc_enc::Patch> newOrder;
        for (const auto& unionPatch : unionPatches) {
            for (const auto& patch : frame->patchList) {
                if (patch.unionPatchReferenceIdx == unionPatch.patchIndex_) {
                    newOrder.push_back(patch);
                    break;
                }
            }
        }

        // Now all matched patches has been pushed, push the non-matched patches, that are already sorted by size.
        for (const auto& patch : frame->patchList) {
            if (!patch.isLinkToAMegaPatch) {
                newOrder.push_back(patch);
            }
        }
        frame->patchList = newOrder;
    }  // TODO(lf): use only index, and made two list of index to the patchList, for matched and non matched -> nop, indeed save at which index
       // we jump from matched to non matched. Then, give to the inter and intra packing function a section of the vector
    // TODO(lf)the previus sorting can be done in place !!!! by sorting only a section of the vector, the first one, with the matched patch.

    // Undo the sorting of the union matches so that their patch id (which is also the unionPatchReferenceIdx for the patch link to it)
    // corresponds to its location in the unionPatches vector.
    std::sort(unionPatches.begin(), unionPatches.end(),
              [](const uvgvpcc_enc::Patch& patchA, const uvgvpcc_enc::Patch& patchB) { return patchA.patchIndex_ < patchB.patchIndex_; });

    // Iterate over each frame and pack first the matched patches (using the union patch position), and then the non-matched patches (using
    // default patch packing method)
    for (auto& frame : gof->frames) {
        // Notice that gof->mapHeightGOF has been updated during the intra pacth packing of the union patches. It is usefull so that if the
        // limit map height has been exceeded during the union patches packing, all new occupancy map of the gof will be allocated using the
        // new map height, not the default minimum limit height.
        
        allocateDefaultOccupancyMap(frame,firstFrame->mapHeight);

        // Separate in two the frame patch list to distinguish the matched and non-matched patches. This symbolic or superficial, no impact on
        // memory.
        std::span<uvgvpcc_enc::Patch> matchedPatches(frame->patchList.begin(),
                                                     frame->patchList.begin() + static_cast<std::ptrdiff_t>(nbUnionPatch));
        std::span<uvgvpcc_enc::Patch> nonMatchedPatches(frame->patchList.begin() + static_cast<std::ptrdiff_t>(nbUnionPatch),
                                                        frame->patchList.end());
        frameInterPatchPacking(unionPatches, frame, &matchedPatches);

        uvgvpcc_enc::Logger::log(uvgvpcc_enc::LogLevel::TRACE, "PATCH PACKING",
                                 "Intra pack patches of the non-matched patches of frame " + std::to_string(frame->frameId) + ".\n");
        frameIntraPatchPacking(frame, &nonMatchedPatches);
    }

    // Debug color //
    /*for(auto& frame : gof->frames) {
        for(auto& patch : frame->patchList) {
            if(patch.isLinkToAMegaPatch) {
                for(int posPatch = 0; posPatch < patch.area_; ++posPatch) {
                    if(patch.depthPCidxL1_[posPatch]!=g_infinitenumber) {
                        frame->pointsAttribute[patch.depthPCidxL1_[posPatch]] = Utils::patchColors[ patch.unionPatchReferenceIdx %
                        Utils::patchColors.size()];
                    }
                    if(patch.depthPCidxL2_[posPatch]!=g_infinitenumber) {
                        frame->pointsAttribute[patch.depthPCidxL2_[posPatch]] = Utils::patchColors[ patch.unionPatchReferenceIdx %
                        Utils::patchColors.size()];
                    }
                }
            } else {
                for(int posPatch = 0; posPatch < patch.area_; ++posPatch) {
                    if(patch.depthPCidxL1_[posPatch]!=g_infinitenumber) {
                        frame->pointsAttribute[patch.depthPCidxL1_[posPatch]] = {0,0,0};
                    }
                    if(patch.depthPCidxL2_[posPatch]!=g_infinitenumber) {
                        frame->pointsAttribute[patch.depthPCidxL2_[posPatch]] = {0,0,0};
                    }
                }
            }
        }
    }*/
}