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

/// \file Intermediary files exportation management.

#include "fileExport.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <limits>
#include <memory>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "parameters.hpp"
#include "utils.hpp"
#include "uvgvpcc/log.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using namespace uvgvpcc_enc;

namespace {

// NOLINTNEXTLINE(cert-err58-cpp)
const std::array<Vector3<uint8_t>, 6> ppiColors = {{{51, 51, 51}, {0, 102, 51}, {153, 0, 0}, {0, 51, 102}, {255, 204, 0}, {102, 204, 204}}};

// NOLINTNEXTLINE(cert-err58-cpp)
const std::array<Vector3<uint8_t>, 114> patchColors = {{
    // Red color is for points not being part of a patch before the 2D projection.
    {139, 0, 0},     {165, 42, 42},   {178, 34, 34},   {220, 20, 60},   {255, 99, 71},   {255, 127, 80},  {205, 92, 92},   {240, 128, 128},
    {233, 150, 122}, {250, 128, 114}, {255, 160, 122}, {255, 69, 0},    {255, 140, 0},   {255, 165, 0},   {255, 215, 0},   {184, 134, 11},
    {218, 165, 32},  {238, 232, 170}, {189, 183, 107}, {240, 230, 140}, {255, 255, 0},   {32, 178, 170},  {0, 128, 128},   {0, 139, 139},
    {0, 255, 255},   {0, 255, 255},   {224, 255, 255}, {0, 206, 209},   {72, 209, 204},  {175, 238, 238}, {176, 224, 230}, {95, 158, 160},
    {70, 130, 180},  {100, 149, 237}, {0, 191, 255},   {30, 144, 255},  {173, 216, 230}, {135, 206, 235}, {135, 206, 250}, {25, 25, 112},
    {0, 0, 128},     {0, 0, 139},     {0, 0, 205},     {0, 0, 255},     {65, 105, 225},  {138, 43, 226},  {75, 0, 130},    {72, 61, 139},
    {106, 90, 205},  {123, 104, 238}, {147, 112, 219}, {139, 0, 139},   {148, 0, 211},   {153, 50, 204},  {186, 85, 211},  {128, 0, 128},
    {216, 191, 216}, {221, 160, 221}, {238, 130, 238}, {255, 0, 255},   {218, 112, 214}, {199, 21, 133},  {219, 112, 147}, {255, 20, 147},
    {255, 105, 180}, {255, 182, 193}, {255, 192, 203}, {250, 235, 215}, {245, 245, 220}, {255, 228, 196}, {255, 235, 205}, {245, 222, 179},
    {255, 248, 220}, {255, 250, 205}, {250, 250, 210}, {255, 255, 224}, {139, 69, 19},   {160, 82, 45},   {210, 105, 30},  {205, 133, 63},
    {244, 164, 96},  {222, 184, 135}, {210, 180, 140}, {188, 143, 143}, {255, 228, 181}, {255, 222, 173}, {255, 218, 185}, {255, 228, 225},
    {255, 240, 245}, {250, 240, 230}, {253, 245, 230}, {255, 239, 213}, {255, 245, 238}, {245, 255, 250}, {112, 128, 144}, {119, 136, 153},
    {176, 196, 222}, {230, 230, 250}, {255, 250, 240}, {240, 248, 255}, {248, 248, 255}, {240, 255, 240}, {255, 255, 240}, {240, 255, 255},
    {255, 250, 250}, {0, 0, 0},       {105, 105, 105}, {128, 128, 128}, {169, 169, 169}, {192, 192, 192}, {211, 211, 211}, {220, 220, 220},
    {245, 245, 245}, {255, 255, 255},
}};

void createDirs(const std::string& filePath) {
    const std::filesystem::path path(filePath);
    auto dir = path.parent_path();
    if (dir.empty()) {
        // The file path is the current directory.
        return;
    }

    if (std::filesystem::exists(dir)) {
        // The output directory already exist.
        return;
    }

    // Thread-safe missing directory creation
    static std::mutex fs_mutex;
    {
        const std::lock_guard<std::mutex> lock(fs_mutex);
        std::error_code ec;
        if (!std::filesystem::create_directories(dir, ec)) {
            if (ec) {
                throw std::runtime_error("Error: failed to create directories: " + dir.string() + " for the intermediate file: " + filePath +
                                         ". " + ec.message());
            }
        }
    }
}

void exportPointCloud(const std::string& filePath, const std::vector<Vector3<typeGeometryInput>>& geometries,
                      const std::vector<Vector3<uint8_t>>& attributes, const std::vector<Vector3<double>>& normals = {}) {
    createDirs(filePath);

    std::ofstream fout(filePath, std::ofstream::out | std::ofstream::trunc);
    if (!fout.is_open()) {
        throw std::runtime_error("Error: Cannot open file for writing: " + filePath);
    }

    // ply header
    const bool hasNormals = !normals.empty();
    fout << "ply";
    fout << "\nformat ascii 1.0";
    fout << "\nelement vertex " << geometries.size();
    fout << "\nproperty int x";
    fout << "\nproperty int y";
    fout << "\nproperty int z";
    if (hasNormals) {
        fout << "\nproperty double nx";
        fout << "\nproperty double ny";
        fout << "\nproperty double nz";
    }
    fout << "\nproperty uchar red";
    fout << "\nproperty uchar green";
    fout << "\nproperty uchar blue";
    fout << "\nend_header\n";

    // ply body
    fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (size_t i = 0; i < geometries.size(); ++i) {
        fout << geometries[i][0] << " " << geometries[i][1] << " " << geometries[i][2];
        if (hasNormals) {
            fout << " " << normals[i][0] << " " << normals[i][1] << " " << normals[i][2];
        }
        fout << " " << static_cast<int>(attributes[i][0]) << " " << static_cast<int>(attributes[i][1]) << " "
             << static_cast<int>(attributes[i][2]);
        fout << std::endl;
    }
    if (!fout) {
        throw std::runtime_error("Error: Failed while writing to file: " + filePath);
    }

    fout.close();
    if (!fout) {
        throw std::runtime_error("Error: Failed to close file after writing: " + filePath);
    }
}

void exportImage(const std::string& filePath, const std::vector<uint8_t>& image, const std::vector<uint8_t>& imageL2 = {}) {
    createDirs(filePath);
    const std::streamsize streamSize = static_cast<std::streamsize>(image.size());

    std::ofstream yuvFile(filePath, std::ios::binary);
    if (!yuvFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + filePath);
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
    yuvFile.write(reinterpret_cast<const char*>(image.data()), streamSize);
    if (!yuvFile) {
        throw std::runtime_error("Error while writing to file: " + filePath);
    }

    if (!imageL2.empty()) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
        yuvFile.write(reinterpret_cast<const char*>(imageL2.data()), streamSize);
        if (!yuvFile) {
            throw std::runtime_error("Error while writing L2 to file: " + filePath);
        }
    }

    yuvFile.close();
    if (!yuvFile) {
        throw std::runtime_error("Error while closing file: " + filePath);
    }
}

void exportBitstream(const std::string& filePath, const std::vector<uint8_t>& bitstream) {
    createDirs(filePath);
    const std::streamsize streamSize = static_cast<std::streamsize>(bitstream.size());

    std::ofstream binaryFile(filePath, std::ios::binary);
    if (!binaryFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + filePath);
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) : lf Accepted for I/O operations
    binaryFile.write(reinterpret_cast<const char*>(bitstream.data()), streamSize);
    if (!binaryFile) {
        throw std::runtime_error("Error while writing to file: " + filePath);
    }

    binaryFile.close();
    if (!binaryFile) {
        throw std::runtime_error("Error while closing file: " + filePath);
    }
}

}  // Anonymous namespace

namespace FileExport {

void cleanIntermediateFiles() {
    // Remove all files within the intermediate files directory if it exists but keep the subdirectory structure intact.
    Logger::log<LogLevel::INFO>("EXPORT FILE", "Clean intermediate files directory.\n");

    const std::filesystem::path root(p_->intermediateFilesDir);

    if (!std::filesystem::exists(root)) {
        // Nothing to do
        return;
    }

    if (!std::filesystem::is_directory(root)) {
        throw std::runtime_error("Error: The given intermediate files directory is not a directory.");
    }

    // Traverse recursively
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (std::filesystem::is_regular_file(entry.path())) {
            std::error_code ec;
            std::filesystem::remove(entry.path(), ec);
            if (ec) {
                throw std::runtime_error("Error: Failed to remove file " + entry.path().string() + "(" + ec.message() + ").");
            }
        }
    }
}

void exportPointCloudNormalComputation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry,
                                       std::vector<Vector3<double>>& normals) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate point cloud after normal computation for frame " + std::to_string(frame->frameId) + ".\n");

    const std::string outputPath = p_->intermediateFilesDir + "/01-normalComputation/NORMAL-COMPUTATION_f" + zeroPad(frame->frameNumber, 3) +
                                   "_vox" + std::to_string(p_->geoBitDepthVoxelized) + ".ply";
    if (p_->geoBitDepthVoxelized == p_->geoBitDepthInput) {
        // If no voxelization
        exportPointCloud(outputPath, frame->pointsGeometry, frame->pointsAttribute, normals);
    } else {
        // If voxelization
        const std::vector<Vector3<uint8_t>> attributes(pointsGeometry.size(), {128, 128, 128});
        exportPointCloud(outputPath, pointsGeometry, attributes, normals);
    }
}

void exportPointCloudNormalOrientation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry,
                                       std::vector<Vector3<double>>& normals) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate point cloud after normal orientation for frame " + std::to_string(frame->frameId) + ".\n");

    const std::string outputPath = p_->intermediateFilesDir + "/02-normalOrientation/NORMAL-ORIENTATION_f" + zeroPad(frame->frameNumber, 3) +
                                   "_vox" + std::to_string(p_->geoBitDepthVoxelized) + ".ply";
    if (p_->geoBitDepthVoxelized == p_->geoBitDepthInput) {
        // If no voxelization
        exportPointCloud(outputPath, frame->pointsGeometry, frame->pointsAttribute, normals);
    } else {
        // If voxelization
        const std::vector<Vector3<uint8_t>> attributes(pointsGeometry.size(), {128, 128, 128});
        exportPointCloud(outputPath, pointsGeometry, attributes, normals);
    }
}

void exportPointCloudInitialSegmentation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry,
                                         const std::vector<size_t>& pointsPPIs) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate point cloud after initial segmentation for frame " + std::to_string(frame->frameId) + ".\n");

    const std::string outputPath = p_->intermediateFilesDir + "/03-initialSegmentation/INITIAL-SEGMENTATION_f" +
                                   zeroPad(frame->frameNumber, 3) + "_vox" + std::to_string(p_->geoBitDepthVoxelized) + ".ply";

    std::vector<Vector3<uint8_t>> attributes(pointsGeometry.size());
    for (size_t pointIndex = 0; pointIndex < pointsGeometry.size(); ++pointIndex) {
        attributes[pointIndex] = ppiColors[pointsPPIs[pointIndex]];
    }
    exportPointCloud(outputPath, pointsGeometry, attributes);
}

void exportPointCloudRefineSegmentation(const std::shared_ptr<Frame>& frame, const std::vector<Vector3<typeGeometryInput>>& pointsGeometry,
                                        const std::vector<size_t>& pointsPPIs) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate point cloud after refine segmentation for frame " + std::to_string(frame->frameId) + ".\n");

    const std::string outputPath = p_->intermediateFilesDir + "/04-refineSegmentation/REFINE-SEGMENTATION_f" +
                                   zeroPad(frame->frameNumber, 3) + "_vox" + std::to_string(p_->geoBitDepthVoxelized) + ".ply";

    std::vector<Vector3<uint8_t>> attributes(pointsGeometry.size());
    for (size_t pointIndex = 0; pointIndex < pointsGeometry.size(); ++pointIndex) {
        attributes[pointIndex] = ppiColors[pointsPPIs[pointIndex]];
    }
    exportPointCloud(outputPath, pointsGeometry, attributes);
}

void exportPointCloudPatchSegmentation(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate point cloud after patch segmentation for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/05-patchSegmentation/PATCH-SEGMENTATION_f" + zeroPad(frame->frameNumber, 3) +
                                   "_vox" + std::to_string(p_->geoBitDepthInput) + ".ply";

    std::vector<Vector3<uint8_t>> attributes(frame->pointsGeometry.size());
    std::vector<bool> pointColored(frame->pointsGeometry.size(), false);
    for (const auto& patch : frame->patchList) {
        const auto color = patchColors[patch.patchIndex_ % patchColors.size()];
        for (size_t v = 0; v < patch.heightInPixel_; ++v) {
            for (size_t u = 0; u < patch.widthInPixel_; ++u) {
                const size_t pos = v * patch.widthInPixel_ + u;
                const typeGeometryInput depth = patch.depthL1_[pos];
                if (depth == g_infiniteDepth) {
                    if (p_->doubleLayer) {
                        assert(patch.depthL2_[pos] == g_infiniteDepth);
                    }
                    continue;
                }
                const size_t ptIndexL1 = patch.depthPCidxL1_[pos];
                attributes[ptIndexL1] = color;
                pointColored[ptIndexL1] = true;

                if (!p_->doubleLayer) continue;
                const size_t ptIndexL2 = patch.depthPCidxL2_[pos];
                if (ptIndexL1 == ptIndexL2) continue;
                attributes[ptIndexL2] = color;
                pointColored[ptIndexL2] = true;
            }
        }
    }

    for (int i = 0; i < frame->pointsGeometry.size(); ++i) {
        if (pointColored[i]) continue;
        // Points that are not within a patch are colored in red
        attributes[i] = Vector3<uint8_t>(255, 0, 0);
    }

    exportPointCloud(outputPath, frame->pointsGeometry, attributes);
}

void exportImageOccupancy(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate occupancy map for frame " + std::to_string(frame->frameId) + ".\n");

    {
        // Export the pristine occupancy map (YUV400)
        const std::string outputPath = p_->intermediateFilesDir + "/06-occupancy/OCCUPANCY_f" + zeroPad(frame->frameNumber, 3) + "_YUV400_" +
                                       std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".yuv";
        exportImage(outputPath, frame->occupancyMap);
    }

    {
        // Export the recolored occupancy map for human viewing (RGB, PNG lossless)
        const size_t imageSize = frame->occupancyMap.size();
        const std::string outputPath = p_->intermediateFilesDir + "/06-occupancyRecolored/OCCUPANCY-RECOLORED_f" +
                                       zeroPad(frame->frameNumber, 3) + "_RGB444_" + std::to_string(p_->mapWidth) + "x" +
                                       std::to_string(frame->mapHeight) + ".rgb";
        std::vector<uint8_t> occupancyMapRecolored(imageSize * 3);
        for (size_t i = 0; i < imageSize; ++i) {
            const uint8_t grayValue = 164 * frame->occupancyMap[i];
            occupancyMapRecolored[i * 3] = grayValue;
            occupancyMapRecolored[i * 3 + 1] = grayValue;
            occupancyMapRecolored[i * 3 + 2] = grayValue;
        }
        exportImage(outputPath, occupancyMapRecolored);
    }
}

void exportImageOccupancyDS(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE",
                                 "Export intermediate downscaled occupancy map for frame " + std::to_string(frame->frameId) + ".\n");

    {
        // Export the pristine occupancy map (YUV420)
        const std::string outputPath = p_->intermediateFilesDir + "/07-occupancyDS/OCCUPANCY-DS_f" + zeroPad(frame->frameNumber, 3) +
                                       "_YUV420_" + std::to_string(p_->mapWidth / p_->occupancyMapDSResolution) + "x" +
                                       std::to_string(frame->mapHeightDS) + ".yuv";
        exportImage(outputPath, frame->occupancyMapDS);
    }

    {
        // Export the recolored occupancy map for human viewing (RGB)
        const size_t imageSize = frame->occupancyMapDS.size();
        const std::string outputPath = p_->intermediateFilesDir + "/07-occupancyDSRecolored/OCCUPANCY-DS-RECOLORED_f" +
                                       zeroPad(frame->frameNumber, 3) + "_RGB444_" +
                                       std::to_string(p_->mapWidth / p_->occupancyMapDSResolution) + "x" +
                                       std::to_string(p_->mapWidth / p_->occupancyMapDSResolution) + ".rgb";
        std::vector<uint8_t> occupancyMapDSRecolored(imageSize * 3);
        for (size_t i = 0; i < imageSize; ++i) {
            const uint8_t grayValue = 164 * frame->occupancyMapDS[i];
            occupancyMapDSRecolored[i * 3] = grayValue;
            occupancyMapDSRecolored[i * 3 + 1] = grayValue;
            occupancyMapDSRecolored[i * 3 + 2] = grayValue;
        }
        exportImage(outputPath, occupancyMapDSRecolored);
    }
}

void exportImageAttribute(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate attribute map for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/08-attribute/ATTRIBUTE_f" + zeroPad(frame->frameNumber, 3) + "_RGB444_" +
                                   std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".rgb";
    if (p_->doubleLayer) {
        exportImage(outputPath, frame->attributeMapL1, frame->attributeMapL2);
    } else {
        exportImage(outputPath, frame->attributeMapL1);
    }
}

void exportImageGeometry(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate geometry map for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/09-geometry/GEOMETRY_f" + zeroPad(frame->frameNumber, 3) + "_YUV420_" +
                                   std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".yuv";
    if (p_->doubleLayer) {
        exportImage(outputPath, frame->geometryMapL1, frame->geometryMapL2);
    } else {
        exportImage(outputPath, frame->geometryMapL1);
    }
}

void exportImageAttributeBgFill(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate attribute map after background filling for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/10-attributeBgFill/ATTRIBUTE-BG-FILL_f" + zeroPad(frame->frameNumber, 3) +
                                   "_RGB444_" + std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".rgb";
    if (p_->doubleLayer) {
        exportImage(outputPath, frame->attributeMapL1, frame->attributeMapL2);
    } else {
        exportImage(outputPath, frame->attributeMapL1);
    }
}

void exportImageGeometryBgFill(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate geometry map after background filling for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/11-geometryBgFill/GEOMETRY-BG-FILL_f" + zeroPad(frame->frameNumber, 3) +
                                   "_YUV420_" + std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".yuv";
    if (p_->doubleLayer) {
        exportImage(outputPath, frame->geometryMapL1, frame->geometryMapL2);
    } else {
        exportImage(outputPath, frame->geometryMapL1);
    }
}

void exportImageAttributeYUV(const std::shared_ptr<Frame>& frame) {
    Logger::log<LogLevel::TRACE>(
        "EXPORT FILE", "Export intermediate attribute map after YUV conversion for frame " + std::to_string(frame->frameId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/12-attributeYUV/ATTRIBUTE-YUV_f" + zeroPad(frame->frameNumber, 3) +
                                   "_YUV420_" + std::to_string(p_->mapWidth) + "x" + std::to_string(frame->mapHeight) + ".yuv";
    if (p_->doubleLayer) {
        exportImage(outputPath, frame->attributeMapL1, frame->attributeMapL2);
    } else {
        exportImage(outputPath, frame->attributeMapL1);
    }
}

void exportOccupancyBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream,
                              const std::string& codecExtension) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate occupancy bitstream for gof " + std::to_string(gof->gofId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/13-occupancyBistream/OCCUPANCY-BITSTREAM_g" + zeroPad(gof->gofId, 3) +
                                   "_YUV420_" + std::to_string(p_->mapWidth / p_->occupancyMapDSResolution) + "x" +
                                   std::to_string(gof->mapHeightDSGOF) + codecExtension;
    exportBitstream(outputPath, bitstream);
}

void exportAttributeBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream,
                              const std::string& codecExtension) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate attribute bitstream for gof " + std::to_string(gof->gofId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/14-attributeBistream/ATTRIBUTE-BITSTREAM_g" + zeroPad(gof->gofId, 3) +
                                   "_YUV420_" + std::to_string(p_->mapWidth) + "x" + std::to_string(gof->mapHeightGOF) + codecExtension;
    exportBitstream(outputPath, bitstream);
}

void exportGeometryBitstream(const std::shared_ptr<uvgvpcc_enc::GOF>& gof, const std::vector<uint8_t>& bitstream,
                             const std::string& codecExtension) {
    Logger::log<LogLevel::TRACE>("EXPORT FILE", "Export intermediate geometry bitstream for gof " + std::to_string(gof->gofId) + ".\n");
    const std::string outputPath = p_->intermediateFilesDir + "/15-geometryBistream/GEOMETRY-BITSTREAM_g" + zeroPad(gof->gofId, 3) +
                                   "_YUV420_" + std::to_string(p_->mapWidth) + "x" + std::to_string(gof->mapHeightGOF) + codecExtension;
    exportBitstream(outputPath, bitstream);
}

//TODO(lf): currently the file is open and close for every log line...
void exportAtlasInformation(const size_t& gofId, const std::string& logLine) {
    const std::string filePath = p_->intermediateFilesDir +
        "/16-atlasInformation/ATLAS_g" + zeroPad(gofId, 3) + ".txt";

    createDirs(filePath);

    std::ofstream textFile(filePath, std::ios::app);
    if (!textFile.is_open()) {
        throw std::runtime_error("Unable to open file: " + filePath);
    }

    textFile << logLine << '\n';
    if (!textFile) {
        throw std::runtime_error("Error while writing to file: " + filePath);
    }

    textFile.close();
    if (!textFile) {
        throw std::runtime_error("Error while closing file: " + filePath);
    }
}


}  // namespace FileExport