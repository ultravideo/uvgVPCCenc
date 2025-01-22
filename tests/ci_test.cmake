message(STATUS "Including CI tests from ci_test.cmake")
# Define the URL and output paths
set(TEST_FILE_URL "https://ultravideo.fi/UVG-VPC/CI_VPCC.zip")
set(TEST_FILE_ARCHIVE "${CMAKE_SOURCE_DIR}/_sequences/archives/VPCC.zip")
set(TEST_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/_sequences/VPCC")
set(TEST_OUTPUT_BITSTREAM_DIR "${CMAKE_BINARY_DIR}/bitstream")
# Download the file if it doesn't exist already
if(NOT EXISTS ${TEST_FILE_ARCHIVE})
    message(STATUS "Downloading test file...")
    file(DOWNLOAD ${TEST_FILE_URL} ${TEST_FILE_ARCHIVE})
endif()
# Extract the archive if it hasn't been extracted yet
if(NOT EXISTS ${TEST_OUTPUT_DIR})
    message(STATUS "Extracting test archive...")
    file(MAKE_DIRECTORY ${TEST_OUTPUT_DIR})
    file(ARCHIVE_EXTRACT INPUT ${TEST_FILE_ARCHIVE} DESTINATION ${TEST_OUTPUT_DIR})
endif()

file(MAKE_DIRECTORY ${TEST_OUTPUT_BITSTREAM_DIR})

message(STATUS "Defining tests in ci_test.cmake")
# Voxel 9 Tests
add_test_macro(
  ReadyForWinter_vox9_fast_RA_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/ReadyForWinter/ply_vox9/ply_xyz_rgb/ReadyForWinter_UVG_vox9_25_0_250_%04d.ply
  2
  0
  1
  1
  presetName=fast,mode=RA,rate=16-22-2
  120
)

add_test_macro(
  ReadyForWinter_vox9_slow_RA_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/ReadyForWinter/ply_vox9/ply_xyz_rgb/ReadyForWinter_UVG_vox9_25_0_250_%04d.ply
  2
  0
  1
  1
  presetName=slow,mode=RA,rate=32-42-4
  120
)

add_test_macro(
  ReadyForWinter_vox9_fast_AI_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/ReadyForWinter/ply_vox9/ply_xyz_rgb/ReadyForWinter_UVG_vox9_25_0_250_%04d.ply
  20
  0
  2
  1
  presetName=fast,mode=AI,rate=32-42-4
  120
)

add_test_macro(
  ReadyForWinter_vox9_slow_AI_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/ReadyForWinter/ply_vox9/ply_xyz_rgb/ReadyForWinter_UVG_vox9_25_0_250_%04d.ply
  2
  0
  1
  1
  presetName=slow,mode=AI,rate=16-22-2
  120
)

add_test_macro(
  FlowerWave_vox10_fast_RA_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/FlowerWave/ply_vox10/ply_xyz_rgb/FlowerWave_UVG_vox10_25_0_250_%04d.ply
  10
  0
  1
  1
  presetName=fast,mode=RA,rate=32-42-4
  120
)

add_test_macro(
  Gymnast_vox10_slow_RA_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/Gymnast/ply_vox10/ply_xyz_rgb/Gymnast_UVG_vox10_25_0_250_%04d.ply
  2
  0
  1
  1
  presetName=slow,mode=RA,rate=16-22-2
  120
)

add_test_macro(
  Gymnast_vox10_fast_AI_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/Gymnast/ply_vox10/ply_xyz_rgb/Gymnast_UVG_vox10_25_0_250_%04d.ply
  18
  0
  1
  1
  presetName=fast,mode=AI,rate=32-42-4
  120
)

add_test_macro(
  FlowerWave_vox10_fast_AI_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/FlowerWave/ply_vox10/ply_xyz_rgb/FlowerWave_UVG_vox10_25_0_250_%04d.ply
  2
  0
  2
  1
  presetName=slow,mode=AI,rate=16-22-2
  120
)

add_test_macro(
  CasualSquat_vox11_fast_RA_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/CasualSquat/ply_vox11/ply_xyz_rgb/CasualSquat_UVG_vox11_25_0_250_%04d.ply
  2
  0
  2
  1
  presetName=fast,mode=RA,rate=16-22-2
  120
)

add_test_macro(
  BlueBackpack_vox11_slow_RA_16-22-2
  ${TEST_OUTPUT_DIR}/UVG-VPC/BlueBackpack/ply_vox11/ply_xyz_rgb/BlueBackpack_UVG_vox11_25_0_250_%04d.ply
  2
  0
  1
  1
  presetName=slow,mode=RA,rate=16-22-2
  120
)

add_test_macro(
  BlueBackpack_vox11_slow_AI_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/BlueBackpack/ply_vox11/ply_xyz_rgb/BlueBackpack_UVG_vox11_25_0_250_%04d.ply
  1
  0
  1
  1
  presetName=fast,mode=AI,rate=32-42-4
  120
)

add_test_macro(
  CasualSquat_vox11_slow_AI_32-42-4
  ${TEST_OUTPUT_DIR}/UVG-VPC/CasualSquat/ply_vox11/ply_xyz_rgb/CasualSquat_UVG_vox11_25_0_250_%04d.ply
  1
  0
  2
  1
  presetName=slow,mode=AI,rate=32-42-4
  120
)
